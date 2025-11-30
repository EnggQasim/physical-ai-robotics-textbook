"""Urdu Translation Service.

Provides:
- On-demand text translation (English → Urdu)
- Translation caching for performance
- Technical term handling (keeping English in parentheses)
"""
import hashlib
from typing import Optional, Dict
import openai

from app.config import get_settings


# In-memory translation cache (production would use Redis or database)
_translation_cache: Dict[str, str] = {}


class TranslationService:
    """Service for English to Urdu translation using GPT-4o-mini."""

    def __init__(self):
        settings = get_settings()
        self.client = openai.OpenAI(api_key=settings.openai_api_key)
        self.model = settings.chat_model  # gpt-4o-mini

    def _get_cache_key(self, text: str) -> str:
        """Generate cache key for text."""
        return hashlib.md5(text.encode()).hexdigest()

    def get_cached_translation(self, text: str) -> Optional[str]:
        """Get translation from cache if available."""
        key = self._get_cache_key(text)
        return _translation_cache.get(key)

    def cache_translation(self, text: str, translation: str) -> None:
        """Store translation in cache."""
        key = self._get_cache_key(text)
        _translation_cache[key] = translation

    async def translate_to_urdu(
        self,
        text: str,
        preserve_technical_terms: bool = True,
        context: Optional[str] = None
    ) -> str:
        """
        Translate English text to Urdu.

        Args:
            text: English text to translate
            preserve_technical_terms: Keep technical terms as "Urdu (English)"
            context: Optional context about the content (e.g., "robotics", "programming")

        Returns:
            Urdu translation
        """
        # Check cache first
        cached = self.get_cached_translation(text)
        if cached:
            return cached

        # Build translation prompt
        system_prompt = """You are an expert English to Urdu translator specializing in technical content about robotics, AI, and programming.

Translation Guidelines:
1. Translate to fluent, natural Urdu using Nastaliq script
2. For technical terms (programming concepts, commands, API names), use this format: اردو ترجمہ (English Term)
   Example: "ROS2 node" → "آر او ایس ٹو نوڈ (ROS2 node)"
3. Keep code snippets, file paths, and commands in English
4. Maintain the original meaning and technical accuracy
5. Use appropriate Urdu punctuation and formatting
6. For acronyms, transliterate and include English: پی آئی ڈی (PID)

Output only the Urdu translation, no explanations."""

        user_prompt = f"Translate the following English text to Urdu:\n\n{text}"

        if context:
            user_prompt = f"Context: This is about {context}\n\n{user_prompt}"

        if not preserve_technical_terms:
            system_prompt = system_prompt.replace(
                'For technical terms (programming concepts, commands, API names), use this format: اردو ترجمہ (English Term)',
                'Translate technical terms fully to Urdu where possible'
            )

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,  # Lower temperature for more consistent translations
                max_tokens=2000
            )

            translation = response.choices[0].message.content.strip()

            # Cache the result
            self.cache_translation(text, translation)

            return translation

        except Exception as e:
            print(f"Translation error: {e}")
            raise

    async def translate_paragraph(
        self,
        paragraph: str,
        context: Optional[str] = None
    ) -> str:
        """
        Translate a paragraph with proper handling of mixed content.

        Preserves:
        - Code blocks (```...```)
        - Inline code (`...`)
        - URLs and file paths
        """
        # For now, translate the entire paragraph
        # Future enhancement: Parse and preserve code blocks
        return await self.translate_to_urdu(
            text=paragraph,
            preserve_technical_terms=True,
            context=context
        )

    def get_cache_stats(self) -> Dict[str, int]:
        """Get translation cache statistics."""
        return {
            "cached_translations": len(_translation_cache),
            "cache_size_bytes": sum(len(v.encode()) for v in _translation_cache.values())
        }

    def clear_cache(self) -> None:
        """Clear the translation cache."""
        global _translation_cache
        _translation_cache = {}


# Singleton instance
_translation_service: Optional[TranslationService] = None


def get_translation_service() -> TranslationService:
    """Get or create translation service singleton."""
    global _translation_service
    if _translation_service is None:
        _translation_service = TranslationService()
    return _translation_service
