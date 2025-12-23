---
id: 001
title: Specification Created
stage: spec
date: 2025-12-22
surface: agent
model: Claude Sonnet 4.5
feature: site-urdu-translation-authoritative
branch: 007-site-urdu-translation-authoritative
user: Claude Code
command: /sp.specify
labels: ["translation", "urdu", "navbar", "docusaurus", "specification"]
links:
  spec: specs/007-site-urdu-translation-authoritative/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/007-site-urdu-translation-authoritative/spec.md
 - specs/007-site-urdu-translation-authoritative/checklists/requirements.md
tests:
 - none
---

## Prompt

```
/sp.specify
ID: 007-site-urdu-translation-authoritative

Purpose:
Correct and unify all previous translation specifications into a single, correct implementation.

Overrides (Deprecated):
- 001-site-urdu-translation
- 005-translate-roman-urdu
- 006-urdu-translation-fix

Problems Identified:
- Partial word-by-word translation
- Mixed Roman Urdu and Urdu behavior
- Incorrect button placement (Hero section)
- Multiple conflicting specs causing inconsistent UI

Final Requirements (Authoritative):

Language:
- Translate English → Urdu (Pakistan) ONLY
- No Roman Urdu
- No partial or word-level translation
- Sentence-level, readable Urdu output

Scope:
- Entire website:
  - Homepage
  - All modules and chapters
  - Sidebar and navigation labels
- Code blocks, commands, file paths MUST remain unchanged

UI Placement (Critical):
- Translation toggle MUST be in the TOP NAVBAR
- Position:
  - Next to GitHub link
  - Next to Light/Dark mode toggle
- Same visual style as navbar controls
- MUST NOT appear in hero section or page body

Behavior:
- One toggle button: English ↔ اردو
- Applies site-wide
- Runtime translation only
- Original `.md/.mdx` files remain unchanged

Spec Hygiene Action:
- Older translation specs are deprecated and ignored
- No new translation specs allowed unless explicitly overriding this one

Build & Validation:
- Must pass:
  - `npm run build`
  - `npm run start`
- No UI duplication
- No broken layout
- Ready for GitHub Pages

Success Criteria:
- Clean navbar with GitHub, theme toggle, and Urdu toggle aligned
- Full, readable Urdu translation across the site
- No mixed-language or partial translation artifacts
```

## Response snapshot

Successfully created the authoritative specification for the Urdu Translation (Pakistan) feature that supersedes all previous translation specifications. The specification addresses all identified problems:

1. **Corrects Translation Method**: Specifies English → Urdu (Pakistan) ONLY with sentence-level translation instead of partial word-by-word translation
2. **Removes Roman Urdu**: Eliminates mixed Roman Urdu and Urdu behavior
3. **Fixes UI Placement**: Requires translation toggle in TOP NAVBAR next to GitHub link and theme toggle, not in hero section
4. **Unifies Approach**: Creates single, correct implementation instead of multiple conflicting specs

The specification includes three prioritized user stories:
- User Story 1 (P1): Navbar Urdu Translation Toggle with proper positioning
- User Story 2 (P1): Complete Sentence Translation to avoid partial conversions
- User Story 3 (P2): Proper Navbar Integration with matching styling

All functional requirements are testable and measurable with clear acceptance scenarios. The specification includes 12 functional requirements covering all critical aspects including:
- Single toggle for English ↔ اردو
- Sentence-level translation with proper grammar
- Top navbar positioning with GitHub link and theme toggle
- RTL layout support for Urdu
- Code preservation during translation
- Site-wide application
- Performance requirements (2-second load time)

Success criteria include measurable outcomes like 2-second toggle response, 100% complete sentence translation, consistent navbar positioning, and successful build validation.

The specification is fully compliant with the project constitution and ready for the planning phase.

## Outcome

- ✅ Impact: Authoritative specification that unifies all previous translation specs
- 🧪 Tests: No automated tests for specification phase
- 📁 Files: spec.md and requirements checklist created
- 🔁 Next prompts: Ready for /sp.plan to create implementation plan
- 🧠 Reflection: Specification addresses all problems identified in previous implementations

## Evaluation notes (flywheel)

- Failure modes observed: None - all requirements clearly specified
- Graders run and results (PASS/FAIL): All checklist items PASS
- Prompt variant (if applicable): Authoritative specification creation
- Next experiment (smallest change to try): Planning the implementation