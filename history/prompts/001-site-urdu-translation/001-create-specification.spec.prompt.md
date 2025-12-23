---
id: 1
title: create-site-urdu-translation-spec
stage: spec
date_iso: 2025-12-22
surface: agent
model: claude-sonnet-4-5-20250929
feature: site-urdu-translation
branch: 001-site-urdu-translation
user: user
command: /sp.specify
labels:
  - specification
  - translation
  - urdu
  - docusaurus
  - localization
links:
  spec: specs/001-site-urdu-translation/spec.md
  plan: null
  tasks: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - specs/001-site-urdu-translation/spec.md
  - specs/001-site-urdu-translation/checklists/requirements.md
tests_yaml:
  - specs/001-site-urdu-translation/spec.md validation
  - Grep search for NEEDS CLARIFICATION markers
---

# Create Site-Wide Urdu Translation Feature Specification

## PROMPT_TEXT
Feature: Site-Wide Translation to Urdu (نافذ کرنا)

Goal:
Translate the entire Docusaurus website into Urdu when the user presses a translation button placed in the top navigation bar.

Correction:
- Translation button MUST be in the top navbar
- Button placed near GitHub link and theme (mode) toggle
- Button must NOT appear in hero section or chapter body

User Experience:
- Navbar button label: اردو میں تبدیل کریں
- One-click toggle: English ↔ Urdu
- Applies to the entire website (all pages, chapters, sidebar text)
- No page reload

Behavior:
- Translates all readable content site-wide
- Code blocks, commands, file paths remain unchanged
- Translation applied dynamically at runtime
- Original `.md/.mdx` files remain unchanged