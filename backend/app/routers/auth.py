"""Authentication API endpoints."""
from fastapi import APIRouter, HTTPException, Request, Response, Depends, Cookie
from fastapi.responses import RedirectResponse
from pydantic import BaseModel, Field
from typing import Optional, Dict, Any, List

from app.services.auth import get_auth_service


router = APIRouter(prefix="/auth", tags=["authentication"])


class AuthStatus(BaseModel):
    """Authentication configuration status."""
    github_configured: bool
    google_configured: bool
    any_configured: bool


class UserInfo(BaseModel):
    """Authenticated user information."""
    user_id: str
    email: Optional[str] = None
    name: Optional[str] = None
    avatar_url: Optional[str] = None
    provider: Optional[str] = None
    created_at: Optional[str] = None
    last_login: Optional[str] = None


class SessionInfo(BaseModel):
    """Session information."""
    authenticated: bool
    user: Optional[UserInfo] = None


class AuthUrlResponse(BaseModel):
    """OAuth authorization URL response."""
    url: str
    provider: str


class SignOutResponse(BaseModel):
    """Sign out response."""
    success: bool
    message: str


@router.get("/status", response_model=AuthStatus)
async def get_auth_status() -> AuthStatus:
    """
    Check which OAuth providers are configured.

    Returns configuration status for GitHub and Google OAuth.
    """
    auth_service = get_auth_service()
    status = auth_service.is_configured()

    return AuthStatus(
        github_configured=status["github"],
        google_configured=status["google"],
        any_configured=status["any"]
    )


@router.get("/session", response_model=SessionInfo)
async def get_session(
    auth_token: Optional[str] = Cookie(None, alias="auth_token")
) -> SessionInfo:
    """
    Get current session information.

    Returns authenticated user info if signed in.
    """
    if not auth_token:
        return SessionInfo(authenticated=False)

    auth_service = get_auth_service()
    user_info = auth_service.verify_token(auth_token)

    if not user_info:
        return SessionInfo(authenticated=False)

    # Get full user data
    user = auth_service.get_user(user_info["user_id"])
    if not user:
        return SessionInfo(authenticated=False)

    return SessionInfo(
        authenticated=True,
        user=UserInfo(
            user_id=user["user_id"],
            email=user.get("email"),
            name=user.get("name"),
            avatar_url=user.get("avatar_url"),
            provider=user.get("provider"),
            created_at=user.get("created_at"),
            last_login=user.get("last_login")
        )
    )


@router.get("/github/url", response_model=AuthUrlResponse)
async def get_github_auth_url(
    redirect_uri: str,
    state: Optional[str] = None
) -> AuthUrlResponse:
    """
    Get GitHub OAuth authorization URL.

    Args:
        redirect_uri: URL to redirect after OAuth (must match GitHub app settings)
        state: Optional state parameter for CSRF protection
    """
    auth_service = get_auth_service()

    if not auth_service.is_configured()["github"]:
        raise HTTPException(
            status_code=503,
            detail="GitHub OAuth not configured. Set GITHUB_CLIENT_ID and GITHUB_CLIENT_SECRET."
        )

    url = auth_service.get_github_auth_url(redirect_uri, state)
    return AuthUrlResponse(url=url, provider="github")


@router.get("/google/url", response_model=AuthUrlResponse)
async def get_google_auth_url(
    redirect_uri: str,
    state: Optional[str] = None
) -> AuthUrlResponse:
    """
    Get Google OAuth authorization URL.

    Args:
        redirect_uri: URL to redirect after OAuth (must match Google Cloud Console)
        state: Optional state parameter for CSRF protection
    """
    auth_service = get_auth_service()

    if not auth_service.is_configured()["google"]:
        raise HTTPException(
            status_code=503,
            detail="Google OAuth not configured. Set GOOGLE_CLIENT_ID and GOOGLE_CLIENT_SECRET."
        )

    url = auth_service.get_google_auth_url(redirect_uri, state)
    return AuthUrlResponse(url=url, provider="google")


@router.get("/github/callback")
async def github_callback(
    code: str,
    state: Optional[str] = None,
    redirect_uri: str = "http://localhost:3000/auth/callback"
) -> Response:
    """
    Handle GitHub OAuth callback.

    Exchanges code for token, creates user/session, returns JWT in cookie.
    """
    auth_service = get_auth_service()

    try:
        result = await auth_service.handle_github_callback(code, redirect_uri)

        # Create response with auth cookie
        response = RedirectResponse(url=redirect_uri.replace("/auth/callback", "/"))
        response.set_cookie(
            key="auth_token",
            value=result["token"],
            httponly=True,
            secure=True,
            samesite="lax",
            max_age=60 * 60 * 24 * 7  # 7 days
        )

        return response

    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/google/callback")
async def google_callback(
    code: str,
    state: Optional[str] = None,
    redirect_uri: str = "http://localhost:3000/auth/callback"
) -> Response:
    """
    Handle Google OAuth callback.

    Exchanges code for token, creates user/session, returns JWT in cookie.
    """
    auth_service = get_auth_service()

    try:
        result = await auth_service.handle_google_callback(code, redirect_uri)

        # Create response with auth cookie
        response = RedirectResponse(url=redirect_uri.replace("/auth/callback", "/"))
        response.set_cookie(
            key="auth_token",
            value=result["token"],
            httponly=True,
            secure=True,
            samesite="lax",
            max_age=60 * 60 * 24 * 7  # 7 days
        )

        return response

    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/signout", response_model=SignOutResponse)
async def sign_out(
    response: Response,
    auth_token: Optional[str] = Cookie(None, alias="auth_token")
) -> SignOutResponse:
    """
    Sign out the current user.

    Clears session and removes auth cookie.
    """
    if auth_token:
        auth_service = get_auth_service()
        user_info = auth_service.verify_token(auth_token)

        if user_info:
            auth_service.sign_out(user_info["session_id"])

    # Clear the auth cookie
    response.delete_cookie(key="auth_token")

    return SignOutResponse(success=True, message="Signed out successfully")


@router.get("/me", response_model=UserInfo)
async def get_current_user(
    auth_token: Optional[str] = Cookie(None, alias="auth_token")
) -> UserInfo:
    """
    Get the current authenticated user's profile.

    Requires authentication.
    """
    if not auth_token:
        raise HTTPException(status_code=401, detail="Not authenticated")

    auth_service = get_auth_service()
    user_info = auth_service.verify_token(auth_token)

    if not user_info:
        raise HTTPException(status_code=401, detail="Invalid or expired token")

    user = auth_service.get_user(user_info["user_id"])
    if not user:
        raise HTTPException(status_code=404, detail="User not found")

    return UserInfo(
        user_id=user["user_id"],
        email=user.get("email"),
        name=user.get("name"),
        avatar_url=user.get("avatar_url"),
        provider=user.get("provider"),
        created_at=user.get("created_at"),
        last_login=user.get("last_login")
    )
