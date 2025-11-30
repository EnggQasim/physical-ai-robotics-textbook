"""Authentication service with OAuth support.

Supports GitHub and Google OAuth providers with JWT session management.
Uses in-memory storage for development, PostgreSQL for production.
"""
import hashlib
import secrets
import json
from datetime import datetime, timedelta
from typing import Optional, Dict, Any, List
from pathlib import Path
import jwt
import httpx

from app.config import get_settings


# User and session storage (in-memory for development)
# In production, use PostgreSQL via database_url
_users: Dict[str, Dict[str, Any]] = {}
_sessions: Dict[str, Dict[str, Any]] = {}
_accounts: Dict[str, Dict[str, Any]] = {}  # provider:provider_id -> user_id mapping


class AuthService:
    """OAuth authentication service."""

    def __init__(self):
        self.settings = get_settings()
        self.jwt_secret = self.settings.jwt_secret or secrets.token_hex(32)
        self.session_expire_days = self.settings.session_expire_days

        # OAuth endpoints
        self.github_auth_url = "https://github.com/login/oauth/authorize"
        self.github_token_url = "https://github.com/login/oauth/access_token"
        self.github_user_url = "https://api.github.com/user"

        self.google_auth_url = "https://accounts.google.com/o/oauth2/v2/auth"
        self.google_token_url = "https://oauth2.googleapis.com/token"
        self.google_user_url = "https://www.googleapis.com/oauth2/v2/userinfo"

    def is_configured(self) -> Dict[str, bool]:
        """Check which OAuth providers are configured."""
        return {
            "github": bool(self.settings.github_client_id and self.settings.github_client_secret),
            "google": bool(self.settings.google_client_id and self.settings.google_client_secret),
            "any": bool(
                (self.settings.github_client_id and self.settings.github_client_secret) or
                (self.settings.google_client_id and self.settings.google_client_secret)
            )
        }

    def get_github_auth_url(self, redirect_uri: str, state: Optional[str] = None) -> str:
        """Get GitHub OAuth authorization URL."""
        if not self.settings.github_client_id:
            raise ValueError("GitHub OAuth not configured")

        state = state or secrets.token_urlsafe(16)
        params = {
            "client_id": self.settings.github_client_id,
            "redirect_uri": redirect_uri,
            "scope": "read:user user:email",
            "state": state
        }
        query = "&".join(f"{k}={v}" for k, v in params.items())
        return f"{self.github_auth_url}?{query}"

    def get_google_auth_url(self, redirect_uri: str, state: Optional[str] = None) -> str:
        """Get Google OAuth authorization URL."""
        if not self.settings.google_client_id:
            raise ValueError("Google OAuth not configured")

        state = state or secrets.token_urlsafe(16)
        params = {
            "client_id": self.settings.google_client_id,
            "redirect_uri": redirect_uri,
            "response_type": "code",
            "scope": "openid email profile",
            "state": state
        }
        query = "&".join(f"{k}={v}" for k, v in params.items())
        return f"{self.google_auth_url}?{query}"

    async def handle_github_callback(
        self,
        code: str,
        redirect_uri: str
    ) -> Dict[str, Any]:
        """Handle GitHub OAuth callback and create/update user."""
        if not self.settings.github_client_id or not self.settings.github_client_secret:
            raise ValueError("GitHub OAuth not configured")

        async with httpx.AsyncClient() as client:
            # Exchange code for access token
            token_response = await client.post(
                self.github_token_url,
                data={
                    "client_id": self.settings.github_client_id,
                    "client_secret": self.settings.github_client_secret,
                    "code": code,
                    "redirect_uri": redirect_uri
                },
                headers={"Accept": "application/json"}
            )
            token_data = token_response.json()

            if "error" in token_data:
                raise Exception(f"GitHub OAuth error: {token_data.get('error_description', token_data['error'])}")

            access_token = token_data["access_token"]

            # Get user info
            user_response = await client.get(
                self.github_user_url,
                headers={
                    "Authorization": f"Bearer {access_token}",
                    "Accept": "application/vnd.github+json"
                }
            )
            github_user = user_response.json()

            # Get user email if not public
            email = github_user.get("email")
            if not email:
                emails_response = await client.get(
                    "https://api.github.com/user/emails",
                    headers={
                        "Authorization": f"Bearer {access_token}",
                        "Accept": "application/vnd.github+json"
                    }
                )
                emails = emails_response.json()
                primary_email = next((e for e in emails if e.get("primary")), None)
                email = primary_email["email"] if primary_email else None

        # Create or update user
        user = self._create_or_update_user(
            provider="github",
            provider_id=str(github_user["id"]),
            email=email,
            name=github_user.get("name") or github_user["login"],
            avatar_url=github_user.get("avatar_url")
        )

        # Create session
        session = self._create_session(user["user_id"])

        return {
            "user": user,
            "session": session,
            "token": self._create_jwt(user, session)
        }

    async def handle_google_callback(
        self,
        code: str,
        redirect_uri: str
    ) -> Dict[str, Any]:
        """Handle Google OAuth callback and create/update user."""
        if not self.settings.google_client_id or not self.settings.google_client_secret:
            raise ValueError("Google OAuth not configured")

        async with httpx.AsyncClient() as client:
            # Exchange code for access token
            token_response = await client.post(
                self.google_token_url,
                data={
                    "client_id": self.settings.google_client_id,
                    "client_secret": self.settings.google_client_secret,
                    "code": code,
                    "redirect_uri": redirect_uri,
                    "grant_type": "authorization_code"
                }
            )
            token_data = token_response.json()

            if "error" in token_data:
                raise Exception(f"Google OAuth error: {token_data.get('error_description', token_data['error'])}")

            access_token = token_data["access_token"]

            # Get user info
            user_response = await client.get(
                self.google_user_url,
                headers={"Authorization": f"Bearer {access_token}"}
            )
            google_user = user_response.json()

        # Create or update user
        user = self._create_or_update_user(
            provider="google",
            provider_id=google_user["id"],
            email=google_user.get("email"),
            name=google_user.get("name"),
            avatar_url=google_user.get("picture")
        )

        # Create session
        session = self._create_session(user["user_id"])

        return {
            "user": user,
            "session": session,
            "token": self._create_jwt(user, session)
        }

    def _create_or_update_user(
        self,
        provider: str,
        provider_id: str,
        email: Optional[str],
        name: Optional[str],
        avatar_url: Optional[str]
    ) -> Dict[str, Any]:
        """Create a new user or update existing one."""
        account_key = f"{provider}:{provider_id}"

        # Check if account already exists
        if account_key in _accounts:
            user_id = _accounts[account_key]["user_id"]
            user = _users[user_id]
            # Update user info
            user["name"] = name or user["name"]
            user["avatar_url"] = avatar_url or user["avatar_url"]
            user["last_login"] = datetime.utcnow().isoformat()
            return user

        # Check if email exists (account linking)
        if email:
            for uid, u in _users.items():
                if u.get("email") == email:
                    # Link this account to existing user
                    _accounts[account_key] = {
                        "account_id": secrets.token_urlsafe(8),
                        "user_id": uid,
                        "provider": provider,
                        "provider_id": provider_id,
                        "created_at": datetime.utcnow().isoformat()
                    }
                    u["last_login"] = datetime.utcnow().isoformat()
                    return u

        # Create new user
        user_id = secrets.token_urlsafe(12)
        user = {
            "user_id": user_id,
            "email": email,
            "name": name,
            "avatar_url": avatar_url,
            "provider": provider,
            "provider_id": provider_id,
            "created_at": datetime.utcnow().isoformat(),
            "last_login": datetime.utcnow().isoformat()
        }
        _users[user_id] = user

        # Create account link
        _accounts[account_key] = {
            "account_id": secrets.token_urlsafe(8),
            "user_id": user_id,
            "provider": provider,
            "provider_id": provider_id,
            "created_at": datetime.utcnow().isoformat()
        }

        return user

    def _create_session(self, user_id: str) -> Dict[str, Any]:
        """Create a new session for a user."""
        session_id = secrets.token_urlsafe(16)
        expires_at = datetime.utcnow() + timedelta(days=self.session_expire_days)

        session = {
            "session_id": session_id,
            "user_id": user_id,
            "expires_at": expires_at.isoformat(),
            "created_at": datetime.utcnow().isoformat()
        }
        _sessions[session_id] = session

        return session

    def _create_jwt(self, user: Dict[str, Any], session: Dict[str, Any]) -> str:
        """Create a JWT token for the session."""
        payload = {
            "sub": user["user_id"],
            "session_id": session["session_id"],
            "name": user.get("name"),
            "email": user.get("email"),
            "avatar_url": user.get("avatar_url"),
            "exp": datetime.utcnow() + timedelta(days=self.session_expire_days),
            "iat": datetime.utcnow()
        }
        return jwt.encode(payload, self.jwt_secret, algorithm="HS256")

    def verify_token(self, token: str) -> Optional[Dict[str, Any]]:
        """Verify a JWT token and return user info."""
        try:
            payload = jwt.decode(token, self.jwt_secret, algorithms=["HS256"])

            # Verify session exists
            session_id = payload.get("session_id")
            if session_id not in _sessions:
                return None

            # Verify user exists
            user_id = payload.get("sub")
            if user_id not in _users:
                return None

            return {
                "user_id": user_id,
                "session_id": session_id,
                "name": payload.get("name"),
                "email": payload.get("email"),
                "avatar_url": payload.get("avatar_url")
            }
        except jwt.ExpiredSignatureError:
            return None
        except jwt.InvalidTokenError:
            return None

    def get_user(self, user_id: str) -> Optional[Dict[str, Any]]:
        """Get user by ID."""
        return _users.get(user_id)

    def sign_out(self, session_id: str) -> bool:
        """End a session."""
        if session_id in _sessions:
            del _sessions[session_id]
            return True
        return False

    def get_session(self, session_id: str) -> Optional[Dict[str, Any]]:
        """Get session by ID."""
        session = _sessions.get(session_id)
        if not session:
            return None

        # Check if expired
        expires_at = datetime.fromisoformat(session["expires_at"])
        if expires_at < datetime.utcnow():
            del _sessions[session_id]
            return None

        return session


# Singleton instance
_auth_service: Optional[AuthService] = None


def get_auth_service() -> AuthService:
    """Get or create auth service singleton."""
    global _auth_service
    if _auth_service is None:
        _auth_service = AuthService()
    return _auth_service
