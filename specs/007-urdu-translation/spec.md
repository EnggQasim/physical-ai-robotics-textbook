# Feature Specification: Urdu Translation

**Feature Branch**: `007-urdu-translation`
**Created**: 2025-11-29
**Status**: Draft
**Input**: Hackathon requirement: "Bilingual Support (Urdu translation) - 50 points bonus"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Switch to Urdu Language (Priority: P1)

A Pakistani student who is more comfortable reading in Urdu wants to read the textbook content in their native language. They click a language toggle and the interface switches to Urdu.

**Why this priority**: Language switching is the core feature. Without it, bilingual support doesn't exist.

**Independent Test**: Can be fully tested by toggling language and verifying UI and content changes to Urdu.

**Acceptance Scenarios**:

1. **Given** user is on any page, **When** they click language selector in header, **Then** they see "English" and "اردو" options
2. **Given** user selects "اردو", **When** selection is made, **Then** interface labels switch to Urdu (navigation, buttons, headers)
3. **Given** Urdu is selected, **When** page displays, **Then** text is right-to-left (RTL) aligned
4. **Given** Urdu mode is active, **When** user navigates to another page, **Then** Urdu preference persists

---

### User Story 2 - Read Translated Chapter Content (Priority: P1)

A learner in Urdu mode opens a chapter. They see the chapter content translated to Urdu while code examples remain in English (code is universal).

**Why this priority**: Content translation is the main value. UI translation alone isn't sufficient for bonus points.

**Independent Test**: Can be tested by reading 3 chapters in Urdu and verifying content is properly translated.

**Acceptance Scenarios**:

1. **Given** user is in Urdu mode, **When** they open a chapter, **Then** headings and body text are in Urdu
2. **Given** chapter has code examples, **When** displayed in Urdu mode, **Then** code remains in English (with Urdu comments if applicable)
3. **Given** chapter has technical terms, **When** displayed, **Then** key terms show "Urdu (English)" format for clarity
4. **Given** translation exists, **When** user reads, **Then** translation is natural Urdu (not machine-translated gibberish)

---

### User Story 3 - Translate Selected Text On-Demand (Priority: P2)

A bilingual user reading in English encounters a difficult passage. They select the text and click "Translate to Urdu" to see that specific section translated.

**Why this priority**: On-demand translation adds flexibility without requiring full pre-translation. Enhances UX.

**Independent Test**: Can be tested by selecting text and verifying translation appears correctly.

**Acceptance Scenarios**:

1. **Given** user is in English mode, **When** they select text (1-500 chars), **Then** they see "Translate to اردو" option
2. **Given** user clicks translate, **When** translation generates, **Then** Urdu translation appears in a tooltip/popover
3. **Given** translation appears, **When** user clicks elsewhere, **Then** popover closes
4. **Given** user selects technical content, **When** translated, **Then** technical terms are preserved with Urdu explanation

---

### User Story 4 - View Both Languages Side-by-Side (Priority: P3)

A learner studying English technical terms wants to see both English and Urdu versions together. They enable side-by-side view to compare.

**Why this priority**: Nice-to-have for language learners. Enhances educational value but not core functionality.

**Independent Test**: Can be tested by enabling side-by-side view and verifying layout.

**Acceptance Scenarios**:

1. **Given** user is on chapter page, **When** they click "Side-by-Side" toggle, **Then** page shows English left, Urdu right
2. **Given** side-by-side is active, **When** user scrolls, **Then** both columns scroll together (synced)
3. **Given** code block appears, **When** displayed, **Then** code appears once (centered) with explanations in both languages
4. **Given** mobile viewport, **When** side-by-side is toggled, **Then** message says "Side-by-side requires desktop view"

---

### Edge Cases

- **Translation not available**: For untranslated content: "اردو ترجمہ جلد آ رہا ہے (Urdu translation coming soon)"
- **Mixed content**: For embedded English terms in Urdu text, don't break words across languages
- **RTL layout issues**: Images and diagrams should not flip in RTL mode
- **Code in RTL**: Code blocks must remain LTR even in RTL context
- **Long Urdu text**: Urdu text may be 20-30% longer than English - layouts must accommodate
- **Font loading**: If Urdu font fails to load, fallback to system Nastaliq font
- **On-demand rate limit**: Max 20 on-demand translations per session to prevent abuse

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide language toggle between English and Urdu
- **FR-002**: System MUST switch interface labels to Urdu when selected
- **FR-003**: System MUST display translated chapter content in Urdu
- **FR-004**: System MUST use RTL (right-to-left) layout for Urdu mode
- **FR-005**: System MUST preserve code examples in English (universal)
- **FR-006**: System MUST persist language preference across sessions
- **FR-007**: System MUST support on-demand text translation (English to Urdu)
- **FR-008**: System MUST show technical terms in both languages for clarity
- **FR-009**: System MUST load appropriate Urdu fonts (Jameel Noori Nastaleeq or similar)
- **FR-010**: System SHOULD provide side-by-side bilingual view on desktop

### Non-Functional Requirements

- **NFR-001**: Language switch MUST apply within 1 second
- **NFR-002**: RTL layout MUST not break any page components
- **NFR-003**: On-demand translation MUST return within 3 seconds
- **NFR-004**: Urdu font MUST load within 2 seconds (or use system fallback)
- **NFR-005**: Translated content MUST be reviewed for accuracy (not raw machine translation)
- **NFR-006**: System MUST work offline once language assets are cached

### Key Entities

- **TranslatedContent**: Urdu version of content (translation_id, content_id, language: ur, translated_text, translator: ai/human, reviewed: boolean, created_at)
- **LanguagePreference**: User's language setting (user_id, language: en/ur, updated_at)
- **OnDemandTranslation**: Real-time translation (translation_id, source_text, target_text, session_id, created_at)
- **InterfaceStrings**: UI labels (string_key, en_text, ur_text)

## Assumptions

- Pre-translate main chapter content using GPT-4o-mini (then human review if time permits)
- Interface strings (buttons, navigation) pre-translated and stored in i18n files
- On-demand translation uses GPT-4o-mini API
- Urdu font loaded from Google Fonts or self-hosted
- Docusaurus i18n plugin used for language switching
- Code examples never translated (programming language is English)
- Technical terms kept in English with Urdu transliteration/explanation

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Language toggle switches interface within 1 second
- **SC-002**: RTL layout renders correctly on all major pages (manual check)
- **SC-003**: At least 2 complete chapters have Urdu translations
- **SC-004**: Code blocks remain LTR and readable in Urdu mode
- **SC-005**: Technical terms are understandable by Urdu-only reader (user testing)
- **SC-006**: On-demand translation returns results within 3 seconds
- **SC-007**: Language preference persists across 5 browser sessions
- **SC-008**: No visual layout breaks when switching between languages

