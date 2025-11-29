# Feature Specification: AI Podcast Generator

**Feature Branch**: `003-podcast-generator`
**Created**: 2025-11-29
**Status**: Draft
**Input**: User description: "Podcast feature like NotebookLM using Higgs Audio for multi-speaker educational audio content"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Generate Chapter Podcast (Priority: P1)

A student wants to listen to chapter content while commuting. They click "Generate Podcast" on a chapter page, and after processing, receive a multi-speaker audio discussion that explains the chapter concepts in an engaging conversational format.

**Why this priority**: This is the core podcast functionality - converting book content to audio. Without this, the feature has no value. Key differentiator from other hackathon submissions.

**Independent Test**: Can be fully tested by generating podcasts for 3 chapters and verifying audio quality, speaker distinction, and content accuracy.

**Acceptance Scenarios**:

1. **Given** a user is on any chapter page, **When** they click "Generate Podcast" button, **Then** they see a progress indicator showing generation status
2. **Given** podcast generation is in progress, **When** user waits, **Then** they see estimated time remaining (based on chapter length)
3. **Given** podcast is generated, **When** user clicks play, **Then** audio plays with clear multi-speaker dialogue
4. **Given** podcast is playing, **When** user listens, **Then** they can distinguish between at least 2 different speaker voices

---

### User Story 2 - Listen to Pre-Generated Podcasts (Priority: P1)

A learner browses the book and sees chapters with podcast icons. They click to play immediately without waiting for generation, as podcasts were pre-generated for popular chapters.

**Why this priority**: Pre-generation ensures instant playback for demo and common use cases. Critical for hackathon demo reliability.

**Independent Test**: Can be tested by accessing 3 pre-generated podcasts and verifying immediate playback.

**Acceptance Scenarios**:

1. **Given** a chapter has a pre-generated podcast, **When** user visits the page, **Then** they see a "Play Podcast" button (not "Generate")
2. **Given** user clicks "Play Podcast", **When** audio loads, **Then** playback starts within 2 seconds
3. **Given** podcast is playing, **When** user navigates to another page, **Then** audio continues playing in mini-player
4. **Given** podcast list is available, **When** user browses, **Then** they see duration and date generated for each

---

### User Story 3 - Control Podcast Playback (Priority: P2)

A student is listening to a chapter podcast but wants to skip ahead or replay a section. They use playback controls to navigate within the audio.

**Why this priority**: Standard audio controls are expected but not core to the feature. Enhances UX but works without advanced controls.

**Independent Test**: Can be tested by using all playback controls and verifying they work correctly.

**Acceptance Scenarios**:

1. **Given** podcast is playing, **When** user clicks pause, **Then** audio pauses and button changes to play
2. **Given** podcast is paused, **When** user clicks play, **Then** audio resumes from same position
3. **Given** podcast is playing, **When** user drags progress bar, **Then** audio seeks to that position
4. **Given** podcast is playing, **When** user clicks +15s/-15s buttons, **Then** audio skips forward/backward

---

### User Story 4 - Download Podcast for Offline (Priority: P3)

A student wants to listen offline during travel. They download the podcast MP3 to their device.

**Why this priority**: Nice-to-have for UX but not essential for hackathon demo. Core value is generation and playback.

**Independent Test**: Can be tested by downloading 2 podcasts and verifying files play in external player.

**Acceptance Scenarios**:

1. **Given** podcast exists for a chapter, **When** user clicks download icon, **Then** MP3 file downloads to their device
2. **Given** download completes, **When** user opens file, **Then** it plays in any standard audio player
3. **Given** multiple chapters have podcasts, **When** user downloads each, **Then** files are named with chapter title

---

### Edge Cases

- **Long chapters**: Chapters over 5000 words show warning: "This chapter is long. Podcast generation may take 3-5 minutes."
- **Generation failure**: Show message: "Podcast generation failed. Please try again." with retry button
- **Already generating**: If user clicks generate while in progress: "Podcast is being generated. Please wait..."
- **No content**: If chapter has no text content: "This chapter doesn't have enough content for a podcast."
- **Audio unavailable**: If audio file is missing: "Podcast temporarily unavailable. Please try again later."
- **Mobile data warning**: On mobile, show: "Podcast is ~15MB. Download on WiFi?" with confirm/cancel

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST generate multi-speaker audio podcasts from chapter text content
- **FR-002**: System MUST use at least 2 distinct speaker voices for conversational format
- **FR-003**: System MUST provide playback controls (play, pause, seek, skip ±15s)
- **FR-004**: System MUST show generation progress with estimated time
- **FR-005**: System MUST allow podcast playback to continue across page navigation (mini-player)
- **FR-006**: System MUST support pre-generated podcasts for instant playback
- **FR-007**: System MUST store generated podcasts for future access (not regenerate each time)
- **FR-008**: System MUST provide download option for offline listening
- **FR-009**: System MUST display podcast duration before playback
- **FR-010**: System MUST handle generation queue (one at a time per user)

### Non-Functional Requirements

- **NFR-001**: Podcast generation MUST complete within 5 minutes for average chapter (2000 words)
- **NFR-002**: Audio playback MUST start within 3 seconds of clicking play
- **NFR-003**: Generated audio MUST be clear and understandable (no major artifacts)
- **NFR-004**: Audio files MUST be under 30MB per chapter (compressed MP3)
- **NFR-005**: System MUST handle 10 concurrent podcast generations without failure
- **NFR-006**: Mini-player MUST not interfere with reading experience on mobile

### Key Entities

- **Podcast**: A generated audio file (podcast_id, chapter_id, audio_url, duration_seconds, transcript, status, created_at)
- **PodcastJob**: Generation queue item (job_id, chapter_id, user_id, status: pending/processing/completed/failed, progress_percent, started_at)
- **Speaker**: Voice configuration (speaker_id, name: "Host"/"Expert", voice_config)

## Assumptions

- Higgs Audio V2 is used for multi-speaker dialogue generation (open source, self-hosted)
- Audio stored in cloud storage (S3/Cloudflare R2) not in database
- Pre-generated podcasts for main chapters (1-4) ready at launch
- Anonymous users can play but not generate podcasts (generation requires session)
- Maximum 3 podcast generations per session to prevent abuse
- Podcast script is auto-generated from chapter content using GPT-4o-mini

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 90% of generated podcasts have clearly distinguishable speakers
- **SC-002**: Podcast generation completes within 5 minutes for 95% of average chapters
- **SC-003**: Audio quality rated "acceptable" or better by 3 test listeners
- **SC-004**: Playback controls (play, pause, seek) work correctly 100% of the time
- **SC-005**: Pre-generated podcasts load and play within 3 seconds
- **SC-006**: Generated content accurately represents chapter topics (manual review)
- **SC-007**: Mini-player persists across 5 page navigations without interruption
- **SC-008**: Download produces valid MP3 playable in VLC, QuickTime, or browser

