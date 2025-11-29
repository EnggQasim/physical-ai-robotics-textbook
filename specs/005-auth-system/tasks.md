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
| **Completed** | **0 tasks** |
| **Remaining** | **20 tasks** |

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

- [ ] T001 Install Better-Auth library and dependencies
- [ ] T002 Configure environment variables (OAuth secrets)
- [ ] T003 Create GitHub OAuth App in developer console
- [ ] T004 Create Google OAuth App in Google Cloud Console

---

## Phase 2: Backend Core

> Authentication service implementation

- [ ] T005 Create user schema and database table (Neon PostgreSQL)
- [ ] T006 Create session schema and management
- [ ] T007 Implement Better-Auth configuration
- [ ] T008 Create GitHub OAuth provider configuration
- [ ] T009 Create authentication API endpoints (sign-in, sign-out, session)
- [ ] T010 Implement secure session cookies (httpOnly)

---

## Phase 3: Frontend Integration

> Auth UI components

- [ ] T011 Create SignInButton component with OAuth options
- [ ] T012 Create UserMenu component (avatar dropdown)
- [ ] T013 Implement auth state management (React context)
- [ ] T014 Add auth components to Docusaurus navbar
- [ ] T015 Create protected route wrapper component

---

## Phase 4: User Story 1 - GitHub OAuth (P1)

> Primary authentication method

- [ ] T016 Implement GitHub OAuth flow with Better-Auth
- [ ] T017 Handle OAuth callback and user creation

---

## Phase 5: User Story 2 - Google OAuth (P2)

> Alternative authentication

- [ ] T018 Add Google OAuth provider to Better-Auth config

---

## Phase 6: User Story 3 - Profile (P2)

> User profile management

- [ ] T019 Create Profile page with user information

---

## Phase 7: User Story 4 - Sign Out (P1)

> Session termination

- [ ] T020 Implement sign out functionality with session cleanup

---

## Phase 8: Deployment

> Production deployment

- [ ] T021 Configure OAuth redirect URLs for production
- [ ] T022 Verify OAuth flows on deployed site
- [ ] T023 Test session persistence across pages

---

## Validation Checklist

Before marking feature complete:

- [ ] GitHub sign-in completes successfully
- [ ] Google sign-in completes successfully
- [ ] User avatar and name displayed when signed in
- [ ] Session persists across page navigation
- [ ] Sign out clears session completely
- [ ] Profile page shows correct user info
- [ ] Anonymous users can access all public content
- [ ] OAuth credentials not exposed in client code
