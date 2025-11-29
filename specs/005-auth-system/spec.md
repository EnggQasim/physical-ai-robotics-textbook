# Feature Specification: Authentication System

**Feature Branch**: `005-auth-system`
**Created**: 2025-11-29
**Status**: Draft
**Input**: Hackathon requirement: "User Authentication (OAuth/GitHub) - 50 points bonus"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Sign In with GitHub (Priority: P1)

A student wants to save their progress and get personalized content. They click "Sign in with GitHub" and are authenticated without creating a new account.

**Why this priority**: OAuth is the primary auth method specified in hackathon. GitHub is standard for developer/student audiences.

**Independent Test**: Can be fully tested by completing GitHub OAuth flow and verifying user session is created.

**Acceptance Scenarios**:

1. **Given** a user is on any page, **When** they click "Sign In" in header, **Then** they see "Sign in with GitHub" option
2. **Given** user clicks GitHub sign in, **When** OAuth flow completes, **Then** they are redirected back to original page as authenticated
3. **Given** user is authenticated, **When** they view header, **Then** they see their GitHub avatar and name
4. **Given** auth succeeds, **When** user is logged in, **Then** their user record is created/updated in database

---

### User Story 2 - Sign In with Google (Priority: P2)

A learner prefers using their Google account. They sign in with Google OAuth as an alternative to GitHub.

**Why this priority**: Provides alternative for users without GitHub accounts. Secondary to primary GitHub flow.

**Independent Test**: Can be tested by completing Google OAuth flow and verifying session creation.

**Acceptance Scenarios**:

1. **Given** user opens sign in modal, **When** they view options, **Then** they see both GitHub and Google sign in buttons
2. **Given** user clicks Google sign in, **When** OAuth completes, **Then** they are authenticated with Google profile
3. **Given** user signed in with Google, **When** they view profile, **Then** they see their Google email and name
4. **Given** same email exists from different provider, **When** user signs in, **Then** accounts are linked automatically

---

### User Story 3 - View and Manage Profile (Priority: P2)

An authenticated user wants to see their profile information and manage their preferences. They access a profile page with their details.

**Why this priority**: Profile management is expected for authenticated users. Required for personalization feature to work.

**Independent Test**: Can be tested by signing in and verifying profile page shows correct information.

**Acceptance Scenarios**:

1. **Given** user is signed in, **When** they click avatar in header, **Then** they see dropdown with "Profile" option
2. **Given** user opens profile, **When** page loads, **Then** they see: name, email, avatar, account created date
3. **Given** user is on profile, **When** they update display name, **Then** change is saved and reflected immediately
4. **Given** user is on profile, **When** they view "Sign out" button and click it, **Then** they are logged out

---

### User Story 4 - Sign Out (Priority: P1)

A user on a shared computer wants to sign out to protect their account. They sign out and their session is ended.

**Why this priority**: Critical security feature. Users must be able to end sessions.

**Independent Test**: Can be tested by signing out and verifying session is terminated.

**Acceptance Scenarios**:

1. **Given** user is signed in, **When** they click avatar dropdown, **Then** they see "Sign Out" option
2. **Given** user clicks sign out, **When** action completes, **Then** they see sign in button instead of avatar
3. **Given** user signed out, **When** they try to access protected features, **Then** they are prompted to sign in
4. **Given** user signed out, **When** they refresh page, **Then** they remain signed out (session cleared)

---

### Edge Cases

- **OAuth popup blocked**: Show message: "Please allow popups for authentication" with instructions
- **OAuth denied**: If user cancels OAuth: "Sign in cancelled. You can continue as a guest or try again."
- **Network error during OAuth**: "Connection error during sign in. Please try again."
- **Session expired**: After 7 days: "Your session has expired. Please sign in again."
- **Already signed in**: If user tries to sign in while authenticated: redirect to profile
- **Account linking conflict**: If emails don't match: "This account uses a different email. Sign in with your original method."

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST support GitHub OAuth 2.0 authentication
- **FR-002**: System MUST support Google OAuth 2.0 authentication
- **FR-003**: System MUST create user record on first sign in
- **FR-004**: System MUST display user avatar and name when authenticated
- **FR-005**: System MUST provide sign out functionality
- **FR-006**: System MUST link accounts with same email address
- **FR-007**: System MUST persist session across browser refreshes
- **FR-008**: System MUST provide profile page with user information
- **FR-009**: System MUST work without authentication (anonymous access allowed)
- **FR-010**: System MUST protect user data (email not exposed publicly)

### Non-Functional Requirements

- **NFR-001**: OAuth flow MUST complete within 10 seconds (excluding user interaction)
- **NFR-002**: Session tokens MUST be stored securely (httpOnly cookies)
- **NFR-003**: Sessions MUST expire after 7 days of inactivity
- **NFR-004**: System MUST handle 100 concurrent sign-in attempts
- **NFR-005**: User data MUST be stored securely (encrypted at rest)
- **NFR-006**: OAuth secrets MUST NOT be exposed in client-side code

### Key Entities

- **User**: Authenticated user (user_id, email, name, avatar_url, provider, provider_id, created_at, last_login)
- **Session**: Active session (session_id, user_id, token_hash, expires_at, created_at)
- **Account**: OAuth account link (account_id, user_id, provider: github/google, provider_account_id)

## Assumptions

- Better-Auth library used for authentication (per ADR-002)
- JWT tokens for session management
- OAuth apps created in GitHub and Google developer consoles
- Anonymous users can use core features (reading, chatbot)
- Authenticated users get: progress tracking, personalization, higher rate limits
- No email/password authentication (OAuth only to reduce complexity)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: GitHub OAuth flow completes successfully 95% of attempts
- **SC-002**: Google OAuth flow completes successfully 95% of attempts
- **SC-003**: User session persists across 10 page navigations
- **SC-004**: Sign out clears session completely (verified by checking localStorage/cookies)
- **SC-005**: Profile page loads correct user information for 100% of authenticated users
- **SC-006**: No OAuth credentials exposed in browser network tab or source code
- **SC-007**: Account linking works for users signing in with same email via different providers
- **SC-008**: Session expiry works correctly after 7 days (tested with mock time)

