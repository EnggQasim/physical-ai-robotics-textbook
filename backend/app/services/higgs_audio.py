"""Higgs Audio V2 integration via HuggingFace Gradio API.

Uses the smola/higgs_audio_v2 Space on HuggingFace for multi-speaker TTS.
"""
import httpx
import base64
import json
from typing import Optional, Dict, Any, List, Tuple
from pathlib import Path
import asyncio

from app.config import get_settings


# Available voice presets from Higgs Audio V2
VOICE_PRESETS = {
    "belinda": "Female voice - warm and professional",
    "broom_salesman": "Male voice - energetic and persuasive",
    "chadwick": "Male voice - deep and authoritative",
    "en_man": "Male voice - neutral English",
    "en_woman": "Female voice - neutral English",
    "mabel": "Female voice - friendly and expressive",
    "vex": "Male voice - dynamic and engaging"
}

# Speaker mapping for podcast HOST/EXPERT dialogue
PODCAST_VOICES = {
    "HOST": "en_woman",      # Female host
    "EXPERT": "chadwick"     # Male expert with authoritative voice
}

# HuggingFace Space URL
HIGGS_AUDIO_SPACE = "https://smola-higgs-audio-v2.hf.space"
GRADIO_API_PREFIX = "/gradio_api"


class HiggsAudioService:
    """Service for generating audio using Higgs Audio V2 via Gradio API."""

    def __init__(self):
        self.base_url = HIGGS_AUDIO_SPACE
        self.api_url = f"{self.base_url}{GRADIO_API_PREFIX}"
        self.timeout = httpx.Timeout(120.0, connect=30.0)  # Long timeout for TTS

    async def _call_gradio_api(
        self,
        endpoint: str,
        data: Dict[str, Any],
        files: Optional[Dict[str, bytes]] = None
    ) -> Dict[str, Any]:
        """Make a call to the Gradio API."""
        async with httpx.AsyncClient(timeout=self.timeout) as client:
            url = f"{self.api_url}{endpoint}"

            if files:
                # Multipart form data for file uploads
                response = await client.post(url, data=data, files=files)
            else:
                response = await client.post(url, json=data)

            response.raise_for_status()
            return response.json()

    async def generate_single_speaker(
        self,
        text: str,
        voice: str = "en_woman",
        temperature: float = 0.7
    ) -> bytes:
        """
        Generate audio for a single speaker using smart-voice template.

        Args:
            text: Text to synthesize
            voice: Voice preset name
            temperature: Generation temperature (0.0-1.0)

        Returns:
            Audio data as bytes (WAV format)
        """
        # Use the /call endpoint with predict
        payload = {
            "data": [
                text,           # Input text
                voice,          # Voice preset
                temperature,    # Temperature
                None            # Optional audio reference (not used)
            ]
        }

        try:
            # Submit the job
            result = await self._call_gradio_api("/call/predict", payload)

            # Get the event_id for polling
            if "event_id" in result:
                event_id = result["event_id"]
                audio_data = await self._poll_result(event_id)
                return audio_data
            else:
                raise Exception("No event_id in response")

        except Exception as e:
            raise Exception(f"Higgs Audio API error: {str(e)}")

    async def _poll_result(self, event_id: str, max_attempts: int = 60) -> bytes:
        """Poll for the result of an async Gradio job."""
        async with httpx.AsyncClient(timeout=self.timeout) as client:
            for attempt in range(max_attempts):
                try:
                    # Check job status
                    response = await client.get(
                        f"{self.api_url}/call/predict/{event_id}"
                    )

                    if response.status_code == 200:
                        # Parse streaming response
                        content = response.text

                        # Gradio returns SSE format, parse the data line
                        for line in content.split("\n"):
                            if line.startswith("data:"):
                                data = json.loads(line[5:].strip())
                                if isinstance(data, list) and len(data) > 0:
                                    # First element is the audio file info
                                    audio_info = data[0]
                                    if isinstance(audio_info, dict) and "url" in audio_info:
                                        # Download the audio file
                                        audio_url = audio_info["url"]
                                        if not audio_url.startswith("http"):
                                            audio_url = f"{self.base_url}{audio_url}"

                                        audio_response = await client.get(audio_url)
                                        audio_response.raise_for_status()
                                        return audio_response.content
                                    elif isinstance(audio_info, dict) and "path" in audio_info:
                                        # File path returned, need to fetch from /file endpoint
                                        file_path = audio_info["path"]
                                        file_url = f"{self.base_url}/file={file_path}"
                                        audio_response = await client.get(file_url)
                                        audio_response.raise_for_status()
                                        return audio_response.content

                    elif response.status_code == 202:
                        # Job still processing
                        await asyncio.sleep(2)
                        continue
                    else:
                        raise Exception(f"Unexpected status: {response.status_code}")

                except httpx.ReadTimeout:
                    await asyncio.sleep(2)
                    continue

            raise Exception("Timeout waiting for audio generation")

    async def generate_multi_speaker(
        self,
        script: str,
        host_voice: str = "en_woman",
        expert_voice: str = "chadwick"
    ) -> Tuple[bytes, List[Dict[str, Any]]]:
        """
        Generate multi-speaker audio from a podcast script.

        Parses HOST:/EXPERT: lines and generates audio for each speaker,
        then concatenates them.

        Args:
            script: Podcast script with HOST:/EXPERT: prefixes
            host_voice: Voice preset for HOST
            expert_voice: Voice preset for EXPERT

        Returns:
            Tuple of (combined audio bytes, segment metadata)
        """
        # Parse the script into segments
        segments = self._parse_script(script)

        # Generate audio for each segment
        audio_segments = []
        segment_info = []

        for i, segment in enumerate(segments):
            speaker = segment["speaker"]
            text = segment["text"]

            # Select voice based on speaker
            voice = host_voice if speaker == "HOST" else expert_voice

            try:
                audio_data = await self.generate_single_speaker(text, voice)
                audio_segments.append(audio_data)
                segment_info.append({
                    "index": i,
                    "speaker": speaker,
                    "voice": voice,
                    "text_preview": text[:100] + "..." if len(text) > 100 else text,
                    "success": True
                })
            except Exception as e:
                segment_info.append({
                    "index": i,
                    "speaker": speaker,
                    "voice": voice,
                    "text_preview": text[:100] + "..." if len(text) > 100 else text,
                    "success": False,
                    "error": str(e)
                })

        # Combine audio segments (simple concatenation for WAV)
        combined_audio = self._combine_audio_segments(audio_segments)

        return combined_audio, segment_info

    def _parse_script(self, script: str) -> List[Dict[str, str]]:
        """Parse a podcast script into speaker segments."""
        segments = []
        current_speaker = None
        current_text = []

        for line in script.split("\n"):
            line = line.strip()
            if not line:
                continue

            # Check for speaker prefix
            if line.startswith("HOST:"):
                if current_speaker and current_text:
                    segments.append({
                        "speaker": current_speaker,
                        "text": " ".join(current_text)
                    })
                current_speaker = "HOST"
                current_text = [line[5:].strip()]
            elif line.startswith("EXPERT:"):
                if current_speaker and current_text:
                    segments.append({
                        "speaker": current_speaker,
                        "text": " ".join(current_text)
                    })
                current_speaker = "EXPERT"
                current_text = [line[7:].strip()]
            elif current_speaker:
                # Continuation of current speaker's text
                current_text.append(line)

        # Add final segment
        if current_speaker and current_text:
            segments.append({
                "speaker": current_speaker,
                "text": " ".join(current_text)
            })

        return segments

    def _combine_audio_segments(self, segments: List[bytes]) -> bytes:
        """
        Combine multiple audio segments into one.

        For WAV files, this requires parsing headers and concatenating PCM data.
        For simplicity, we just concatenate and the first segment's header is used.
        """
        if not segments:
            return b""

        if len(segments) == 1:
            return segments[0]

        # For now, return concatenated bytes
        # A proper implementation would use pydub or similar
        # to properly combine WAV files
        return b"".join(segments)

    def get_available_voices(self) -> Dict[str, str]:
        """Get available voice presets and their descriptions."""
        return VOICE_PRESETS.copy()

    async def check_availability(self) -> bool:
        """Check if the Higgs Audio Space is available."""
        try:
            async with httpx.AsyncClient(timeout=httpx.Timeout(10.0)) as client:
                response = await client.get(f"{self.base_url}/")
                return response.status_code == 200
        except Exception:
            return False


# Singleton instance
_higgs_service: Optional[HiggsAudioService] = None


def get_higgs_audio_service() -> HiggsAudioService:
    """Get or create Higgs Audio service singleton."""
    global _higgs_service
    if _higgs_service is None:
        _higgs_service = HiggsAudioService()
    return _higgs_service
