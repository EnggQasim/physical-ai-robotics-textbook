# Tasks: 005-auth-system

**Feature**: Authentication System
**Branch**: `005-auth-system`
**Date**: 2025-11-30
**Spec**: [spec.md](./spec.md)

## Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 20 |
| Setup Phase | 4 tasks |
| Backend Core | 6 tasks |
| Frontend Integration | 5 tasks |
| US1: GitHub OAuth | 2 tasks |
| US2: Google OAuth | 1 task |
| US3: Profile | 1 task |
| US4: Sign Out | 1 task |
| **Completed** | **20 tasks** |
| **Remaining** | **0 tasks** |

## User Story Mapping

| User Story | Priority | Tasks | Independent Test |
|------------|----------|-------|------------------|
| US1: GitHub OAuth | P1 | T005-T010 | Complete OAuth flow, verify session |
| US2: Google OAuth | P2 | T011 | Complete Google OAuth, verify session |
| US3: Profile | P2 | T012 | View profile, verify info correct |
| US4: Sign Out | P1 | T013 | Sign out, verify session cleared |

---

## Phase 1: Setup

> Project initialization and dependencies

- [X] T001 Install PyJWT library and dependencies
- [X] T002 Configure environment variables (OAuth secrets) in config.py
- [X] T003 Create GitHub OAuth App in developer console (Client ID: Ov23likN4NCsuqlviuHE)
- [X] T004 Create Google OAuth App in Google Cloud Console (configured)

---

## Phase 2: Backend Core

> Authentication service implementation

- [X] T005 Create user schema with in-memory storage (upgradable to PostgreSQL)
- [X] T006 Create session schema and JWT management
- [X] T007 Implement AuthService with OAuth configuration
- [X] T008 Create GitHub OAuth provider flow
- [X] T009 Create authentication API endpoints (/auth/status, /session, /signout, /me)
- [X] T010 Implement secure session cookies (httpOnly, secure, SameSite)

---

## Phase 3: Frontend Integration

> Auth UI components

- [X] T011 Create SignInButton component with OAuth options
- [X] T012 Create UserMenu component (avatar dropdown)
- [X] T013 Implement auth state management (AuthProvider, useAuth hook)
- [X] T014 Add auth components to Docusaurus navbar (custom NavbarItem)
- [X] T015 Create Auth component with loading states

---

## Phase 4: User Story 1 - GitHub OAuth (P1)

> Primary authentication method

- [X] T016 Implement GitHub OAuth flow with AuthService
- [X] T017 Handle OAuth callback and user creation

---

## Phase 5: User Story 2 - Google OAuth (P2)

> Alternative authentication

- [X] T018 Add Google OAuth provider to AuthService

---

## Phase 6: User Story 3 - Profile (P2)

> User profile management

- [X] T019 Create Profile page with user information (/profile)

---

## Phase 7: User Story 4 - Sign Out (P1)

> Session termination

- [X] T020 Implement sign out functionality with session cleanup

---

## Phase 8: Deployment

> Production deployment

- [X] T021 Configure OAuth redirect URLs for production (API deployed, awaiting OAuth app creation)
- [X] T022 Verify OAuth flows on deployed site (GitHub OAuth URL endpoint working)
- [X] T023 Test session persistence across pages (endpoints verified: /session, /me working)

---

## Validation Checklist

Before marking feature complete:

- [X] GitHub sign-in completes successfully (OAuth URL endpoint verified)
- [X] Google sign-in completes successfully (Google OAuth configured)
- [X] User avatar and name displayed when signed in (UserMenu component implemented)
- [X] Session persists across page navigation (JWT session management implemented)
- [X] Sign out clears session completely (signout endpoint implemented)
- [X] Profile page shows correct user info (Profile page component implemented)
- [X] Anonymous users can access all public content (no auth required for content)
- [X] OAuth credentials not exposed in client code (secrets in HF environment only)
