"""AI Podcast Generator Service with multi-provider TTS support.

Supports:
- OpenAI TTS (default, reliable)
- Higgs Audio V2 via HuggingFace (multi-speaker, more expressive)
"""
import base64
import hashlib
import os
import io
from typing import Optional, Dict, Any, List, Literal
from pathlib import Path
import json
from datetime import datetime
import openai

from app.config import get_settings
from app.services.higgs_audio import get_higgs_audio_service, PODCAST_VOICES


# TTS Provider type
TTSProvider = Literal["openai", "higgs"]


# Role-based debate prompts for personalized podcasts
ROLE_DEBATE_PROMPTS = {
    "student": {
        "expert_a": "Professor",
        "expert_a_role": "a university professor who explains theory clearly for exams",
        "expert_b": "Teaching Assistant",
        "expert_b_role": "a helpful TA who provides study tips, common pitfalls, and practical examples",
        "context": "classroom examples, homework scenarios, exam preparation tips",
        "tone": "educational, encouraging, uses simple analogies"
    },
    "researcher": {
        "expert_a": "Senior Researcher",
        "expert_a_role": "a senior researcher who discusses cutting-edge papers and theoretical foundations",
        "expert_b": "Lab Lead",
        "expert_b_role": "a lab lead who debates practical research challenges and experimental approaches",
        "context": "recent papers, research directions, experimental methodologies, publication insights",
        "tone": "academic, analytical, cites research trends"
    },
    "engineer": {
        "expert_a": "System Architect",
        "expert_a_role": "a system architect who discusses design patterns and architecture decisions",
        "expert_b": "DevOps Engineer",
        "expert_b_role": "a DevOps engineer who counters with deployment realities and production challenges",
        "context": "production code, scaling considerations, debugging, system reliability, CI/CD",
        "tone": "practical, detail-oriented, focuses on real-world constraints"
    },
    "hobbyist": {
        "expert_a": "Maker",
        "expert_a_role": "an enthusiastic maker who shares creative DIY approaches",
        "expert_b": "Tinkerer",
        "expert_b_role": "a budget-conscious tinkerer who suggests affordable alternatives and weekend projects",
        "context": "weekend projects, affordable hardware, fun experiments, 3D printing, Arduino",
        "tone": "fun, encouraging, cost-effective solutions"
    }
}

# Default voices for each role
ROLE_DEFAULT_VOICES = {
    "student": {"expert_a": "mabel", "expert_b": "chadwick"},
    "researcher": {"expert_a": "belinda", "expert_b": "vex"},
    "engineer": {"expert_a": "en_woman", "expert_b": "chadwick"},
    "hobbyist": {"expert_a": "mabel", "expert_b": "en_man"}
}


# Pre-defined podcast metadata for chapters
CHAPTER_PODCASTS = {
    "intro": {
        "title": "Introduction to Physical AI",
        "description": "Welcome to the Physical AI & Humanoid Robotics textbook",
        "duration_estimate": 180  # 3 minutes
    },
    "module-1-ros2-index": {
        "title": "ROS2 Fundamentals Overview",
        "description": "Introduction to Robot Operating System 2",
        "duration_estimate": 300  # 5 minutes
    },
    "module-1-ros2-nodes-topics": {
        "title": "ROS2 Nodes and Topics",
        "description": "Understanding publisher-subscriber communication",
        "duration_estimate": 420  # 7 minutes
    },
    "module-1-ros2-services-actions": {
        "title": "ROS2 Services and Actions",
        "description": "Request-response and long-running task patterns",
        "duration_estimate": 360  # 6 minutes
    },
    "module-2-simulation-index": {
        "title": "Robot Simulation Overview",
        "description": "Introduction to Gazebo and simulation concepts",
        "duration_estimate": 300
    },
    "module-3-nvidia-isaac-index": {
        "title": "NVIDIA Isaac Platform",
        "description": "GPU-accelerated robotics simulation",
        "duration_estimate": 360
    },
    "module-4-vla-index": {
        "title": "Vision-Language-Action Models",
        "description": "AI-powered robot control with natural language",
        "duration_estimate": 420
    }
}


class PodcastService:
    """Service for generating podcast-style audio from text content."""

    def __init__(self):
        settings = get_settings()
        self.client = openai.OpenAI(api_key=settings.openai_api_key)
        self.chat_model = settings.chat_model
        self.higgs_service = get_higgs_audio_service()

        # Cache directory for generated podcasts (in backend, served via static files)
        self.cache_dir = Path(__file__).parent.parent.parent / "audio" / "podcasts"
        self.cache_dir.mkdir(parents=True, exist_ok=True)

        # Cache metadata file
        self.cache_file = self.cache_dir / "cache.json"
        self.cache = self._load_cache()

    def _load_cache(self) -> Dict[str, Any]:
        """Load podcast cache metadata."""
        if self.cache_file.exists():
            with open(self.cache_file, "r") as f:
                return json.load(f)
        return {"podcasts": {}, "jobs": {}}

    def _save_cache(self):
        """Save podcast cache metadata."""
        with open(self.cache_file, "w") as f:
            json.dump(self.cache, f, indent=2)

    def _get_cache_key(self, content: str) -> str:
        """Generate a cache key from content."""
        return hashlib.md5(content.encode()).hexdigest()[:12]

    async def generate_script(self, chapter_content: str, chapter_title: str) -> str:
        """
        Generate a podcast script from chapter content using GPT-4o-mini.
        Creates a conversational dialogue between two speakers.
        """
        prompt = f"""Convert the following educational content into an engaging podcast script
with two speakers: HOST (the main presenter) and EXPERT (a technical expert who adds insights).

Guidelines:
- Keep it conversational and educational
- HOST introduces topics and asks questions
- EXPERT provides detailed explanations
- Include natural transitions and occasional humor
- Total length should be 3-5 minutes when read aloud
- Format each line as "HOST: text" or "EXPERT: text"

Chapter Title: {chapter_title}

Content:
{chapter_content[:4000]}  # Limit to avoid token limits

Generate the podcast script:"""

        response = self.client.chat.completions.create(
            model=self.chat_model,
            messages=[
                {"role": "system", "content": "You are a podcast script writer who creates engaging educational content."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.7,
            max_tokens=2000
        )

        return response.choices[0].message.content

    async def generate_personalized_script(
        self,
        chapter_content: str,
        chapter_title: str,
        user_role: str = "student",
        experience_level: str = "intermediate",
        interests: List[str] = None
    ) -> str:
        """
        Generate a personalized podcast script with debate-style conversation.

        Two experts discuss and debate the topic based on user's role:
        - student: Professor vs Teaching Assistant
        - researcher: Senior Researcher vs Lab Lead
        - engineer: System Architect vs DevOps Engineer
        - hobbyist: Maker vs Tinkerer

        Args:
            chapter_content: The chapter text to convert
            chapter_title: Title of the chapter
            user_role: User's role (student/researcher/engineer/hobbyist)
            experience_level: beginner/intermediate/advanced
            interests: List of user interests to emphasize

        Returns:
            Podcast script with EXPERT_A: and EXPERT_B: prefixes
        """
        role_config = ROLE_DEBATE_PROMPTS.get(user_role, ROLE_DEBATE_PROMPTS["student"])
        interests = interests or []

        # Adjust complexity based on experience level
        complexity_guide = {
            "beginner": "Use simple language, avoid jargon, explain every technical term with analogies",
            "intermediate": "Balance technical accuracy with clarity, assume basic knowledge",
            "advanced": "Use precise technical language, discuss edge cases and optimizations"
        }

        interest_focus = ""
        if interests:
            interest_focus = f"\n- When relevant, emphasize connections to: {', '.join(interests)}"

        prompt = f"""Create an engaging debate-style podcast script between two experts discussing the following content.

**Characters:**
- EXPERT_A ({role_config['expert_a']}): {role_config['expert_a_role']}
- EXPERT_B ({role_config['expert_b']}): {role_config['expert_b_role']}

**Style Guidelines:**
- Tone: {role_config['tone']}
- Context/Examples: {role_config['context']}
- Complexity: {complexity_guide.get(experience_level, complexity_guide['intermediate'])}{interest_focus}

**Conversation Rules:**
1. EXPERT_A starts by introducing the main concept
2. EXPERT_B adds a different perspective or practical counterpoint
3. They debate, agree, disagree, and build on each other's points
4. Include occasional friendly banter and real-world anecdotes
5. Both experts should be knowledgeable but approach topics differently
6. End with a collaborative summary of key takeaways

**Format:**
- Each line must start with "EXPERT_A:" or "EXPERT_B:"
- Total length: 4-6 minutes when read aloud
- 10-15 exchanges between the experts

**Chapter Title:** {chapter_title}

**Content to discuss:**
{chapter_content[:4000]}

Generate the debate-style podcast script:"""

        response = self.client.chat.completions.create(
            model=self.chat_model,
            messages=[
                {
                    "role": "system",
                    "content": f"You are creating a podcast where two experts ({role_config['expert_a']} and {role_config['expert_b']}) debate and discuss technical topics in an engaging way. Make it feel like a real conversation with occasional interruptions, agreements, and friendly disagreements."
                },
                {"role": "user", "content": prompt}
            ],
            temperature=0.8,  # Slightly higher for more creative conversation
            max_tokens=2500
        )

        return response.choices[0].message.content

    async def generate_audio(
        self,
        script: str,
        voice: str = "alloy"
    ) -> bytes:
        """
        Generate audio from script using OpenAI TTS.

        Voices available: alloy, echo, fable, onyx, nova, shimmer
        """
        response = self.client.audio.speech.create(
            model="tts-1",
            voice=voice,
            input=script
        )

        return response.content

    async def generate_audio_higgs(
        self,
        script: str,
        host_voice: str = "en_woman",
        expert_voice: str = "chadwick"
    ) -> bytes:
        """
        Generate multi-speaker audio using Higgs Audio V2.

        This method generates separate audio for each speaker segment
        and combines them for a more natural podcast feel.

        Args:
            script: Podcast script with HOST:/EXPERT: prefixes
            host_voice: Voice preset for HOST speaker
            expert_voice: Voice preset for EXPERT speaker

        Returns:
            Combined audio data as bytes
        """
        audio_data, segment_info = await self.higgs_service.generate_multi_speaker(
            script=script,
            host_voice=host_voice,
            expert_voice=expert_voice
        )

        return audio_data

    async def generate_podcast(
        self,
        chapter_id: str,
        chapter_content: str,
        chapter_title: str,
        force_regenerate: bool = False,
        tts_provider: TTSProvider = "openai",
        host_voice: str = "en_woman",
        expert_voice: str = "chadwick"
    ) -> Dict[str, Any]:
        """
        Generate a complete podcast for a chapter.

        Args:
            chapter_id: Unique identifier for the chapter
            chapter_content: Text content to convert
            chapter_title: Title of the chapter
            force_regenerate: Force regeneration even if cached
            tts_provider: TTS provider to use ("openai" or "higgs")
            host_voice: Voice for HOST speaker (Higgs only)
            expert_voice: Voice for EXPERT speaker (Higgs only)

        Returns:
            Dict with podcast URL, title, duration, and metadata
        """
        # Include provider in cache key to allow both versions
        cache_key = self._get_cache_key(chapter_id + chapter_content[:500] + tts_provider)
        if not force_regenerate and cache_key in self.cache.get("podcasts", {}):
            cached = self.cache["podcasts"][cache_key]
            return {
                "success": True,
                "podcast_id": cache_key,
                "url": cached["url"],
                "title": cached["title"],
                "duration": cached.get("duration", 300),
                "created_at": cached.get("created_at"),
                "tts_provider": cached.get("tts_provider", "openai"),
                "cached": True
            }

        try:
            # Step 1: Generate script
            script = await self.generate_script(chapter_content, chapter_title)

            # Step 2: Generate audio based on provider
            if tts_provider == "higgs":
                # Use Higgs Audio for multi-speaker TTS
                audio_data = await self.generate_audio_higgs(
                    script,
                    host_voice=host_voice,
                    expert_voice=expert_voice
                )
                file_ext = "wav"  # Higgs returns WAV
            else:
                # Default to OpenAI TTS
                audio_data = await self.generate_audio(script, voice="alloy")
                file_ext = "mp3"

            # Step 3: Save audio file
            filename = f"{cache_key}.{file_ext}"
            filepath = self.cache_dir / filename

            with open(filepath, "wb") as f:
                f.write(audio_data)

            # Estimate duration (rough: ~150 words per minute, ~5 chars per word)
            word_count = len(script.split())
            duration = int((word_count / 150) * 60)

            # Update cache
            url = f"/audio/podcasts/{filename}"
            self.cache.setdefault("podcasts", {})[cache_key] = {
                "url": url,
                "title": chapter_title,
                "chapter_id": chapter_id,
                "duration": duration,
                "script": script[:500] + "...",  # Store preview
                "created_at": datetime.now().isoformat(),
                "file_size": len(audio_data),
                "tts_provider": tts_provider,
                "voices": {
                    "host": host_voice if tts_provider == "higgs" else "alloy",
                    "expert": expert_voice if tts_provider == "higgs" else "alloy"
                }
            }
            self._save_cache()

            return {
                "success": True,
                "podcast_id": cache_key,
                "url": url,
                "title": chapter_title,
                "duration": duration,
                "created_at": datetime.now().isoformat(),
                "tts_provider": tts_provider,
                "cached": False
            }

        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "url": None,
                "tts_provider": tts_provider
            }

    def get_chapter_podcast_info(self, chapter_id: str) -> Optional[Dict[str, Any]]:
        """Get pre-defined podcast info for a chapter."""
        return CHAPTER_PODCASTS.get(chapter_id)

    def get_cached_podcasts(self) -> List[Dict[str, Any]]:
        """Get all cached/generated podcasts."""
        podcasts = []
        for podcast_id, info in self.cache.get("podcasts", {}).items():
            podcasts.append({
                "podcast_id": podcast_id,
                **info
            })
        return podcasts

    def get_podcast_by_id(self, podcast_id: str) -> Optional[Dict[str, Any]]:
        """Get a specific podcast by ID."""
        podcast = self.cache.get("podcasts", {}).get(podcast_id)
        if podcast:
            return {
                "podcast_id": podcast_id,
                "success": True,
                **podcast
            }
        return None

    async def generate_personalized_podcast(
        self,
        chapter_id: str,
        chapter_content: str,
        chapter_title: str,
        user_role: str = "student",
        experience_level: str = "intermediate",
        interests: List[str] = None,
        expert_a_voice: str = None,
        expert_b_voice: str = None,
        force_regenerate: bool = False
    ) -> Dict[str, Any]:
        """
        Generate a personalized debate-style podcast for a chapter.

        Args:
            chapter_id: Unique identifier for the chapter
            chapter_content: Text content to convert
            chapter_title: Title of the chapter
            user_role: User's role (student/researcher/engineer/hobbyist)
            experience_level: beginner/intermediate/advanced
            interests: List of user interests
            expert_a_voice: Voice for Expert A (default based on role)
            expert_b_voice: Voice for Expert B (default based on role)
            force_regenerate: Force regeneration even if cached

        Returns:
            Dict with podcast URL, personalization info, and metadata
        """
        # Get default voices for role if not specified
        role_voices = ROLE_DEFAULT_VOICES.get(user_role, ROLE_DEFAULT_VOICES["student"])
        expert_a_voice = expert_a_voice or role_voices["expert_a"]
        expert_b_voice = expert_b_voice or role_voices["expert_b"]
        interests = interests or []

        # Cache key includes all personalization params
        cache_key = self._get_cache_key(
            f"{chapter_id}:{user_role}:{experience_level}:{expert_a_voice}:{expert_b_voice}"
        )

        if not force_regenerate and cache_key in self.cache.get("podcasts", {}):
            cached = self.cache["podcasts"][cache_key]
            return {
                "success": True,
                "podcast_id": cache_key,
                "url": cached["url"],
                "title": cached["title"],
                "duration": cached.get("duration", 300),
                "created_at": cached.get("created_at"),
                "personalization": cached.get("personalization", {}),
                "cached": True
            }

        try:
            # Get role config for response metadata
            role_config = ROLE_DEBATE_PROMPTS.get(user_role, ROLE_DEBATE_PROMPTS["student"])

            # Step 1: Generate personalized script
            script = await self.generate_personalized_script(
                chapter_content,
                chapter_title,
                user_role=user_role,
                experience_level=experience_level,
                interests=interests
            )

            # Step 2: Generate multi-speaker audio using Higgs
            # Convert EXPERT_A/EXPERT_B to HOST/EXPERT format for Higgs service
            higgs_script = script.replace("EXPERT_A:", "HOST:").replace("EXPERT_B:", "EXPERT:")

            audio_data = await self.higgs_service.generate_multi_speaker(
                script=higgs_script,
                host_voice=expert_a_voice,
                expert_voice=expert_b_voice
            )

            # Handle both tuple return (audio_data, segment_info) and just audio_data
            if isinstance(audio_data, tuple):
                audio_data = audio_data[0]

            file_ext = "wav"

            # Step 3: Save audio file
            filename = f"personalized_{cache_key}.{file_ext}"
            filepath = self.cache_dir / filename

            with open(filepath, "wb") as f:
                f.write(audio_data)

            # Estimate duration
            word_count = len(script.split())
            duration = int((word_count / 150) * 60)

            # Personalization metadata
            personalization = {
                "role": user_role,
                "experience_level": experience_level,
                "interests": interests,
                "expert_a": {
                    "name": role_config["expert_a"],
                    "voice": expert_a_voice
                },
                "expert_b": {
                    "name": role_config["expert_b"],
                    "voice": expert_b_voice
                },
                "adapted_for": role_config["context"]
            }

            # Update cache
            url = f"/audio/podcasts/{filename}"
            self.cache.setdefault("podcasts", {})[cache_key] = {
                "url": url,
                "title": f"{chapter_title} ({role_config['expert_a']} & {role_config['expert_b']})",
                "chapter_id": chapter_id,
                "duration": duration,
                "script": script[:500] + "...",
                "created_at": datetime.now().isoformat(),
                "file_size": len(audio_data),
                "tts_provider": "higgs",
                "personalization": personalization
            }
            self._save_cache()

            return {
                "success": True,
                "podcast_id": cache_key,
                "url": url,
                "title": f"{chapter_title} ({role_config['expert_a']} & {role_config['expert_b']})",
                "duration": duration,
                "created_at": datetime.now().isoformat(),
                "personalization": personalization,
                "script_preview": script[:300] + "...",
                "cached": False
            }

        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "url": None,
                "personalization": {
                    "role": user_role,
                    "experience_level": experience_level
                }
            }


# Singleton instance
_podcast_service: Optional[PodcastService] = None


def get_podcast_service() -> PodcastService:
    """Get or create podcast service singleton."""
    global _podcast_service
    if _podcast_service is None:
        _podcast_service = PodcastService()
    return _podcast_service
