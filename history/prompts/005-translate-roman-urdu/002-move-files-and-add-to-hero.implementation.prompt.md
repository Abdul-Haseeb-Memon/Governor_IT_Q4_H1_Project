---
id: 2
title: move-files-and-add-to-hero
stage: implementation
date_iso: 2025-12-22
surface: agent
model: claude-sonnet-4-5-20250929
feature: translate-roman-urdu
branch: 005-translate-roman-urdu
user: user
command: /sp.implement
labels:
  - implementation
  - translation
  - docusaurus
  - roman-urdu
  - correction
links:
  spec: specs/005-translate-roman-urdu/spec.md
  plan: specs/005-translate-roman-urdu/plan.md
  tasks: specs/005-translate-roman-urdu/tasks.md
  ticket: null
  adr: null
  pr: null
files_yaml:
  - frontend_H_book/src/components/TranslationToggle/TranslationToggle.jsx
  - frontend_H_book/src/components/TranslationToggle/romanUrduTranslator.js
  - frontend_H_book/src/components/TranslationToggle/TranslationToggle.module.css
  - frontend_H_book/src/utils/domUtils.js
  - frontend_H_book/src/components/Homepage/HeroSection.jsx
tests_yaml:
  - Files moved to correct location
  - TranslationToggle added to hero section
  - Import paths verified
---

# Correction: Move Files to Correct Location and Add Translation Button to Hero Section

## PROMPT_TEXT
you make a mistake you add src file in D:\g-house-project not in D:\g-house-project\frontend_H_book> AND ASLO ADD THE BUTTON TRANLATION TO URSRI  ON TO THE HERO SECTION

## RESPONSE_TEXT
Corrected the implementation by:

1. MOVING FILES TO CORRECT LOCATION:
- Moved TranslationToggle component from D:\g-house-project\src\components\TranslationToggle to D:\g-house-project\frontend_H_book\src\components\TranslationToggle
- Moved domUtils.js from D:\g-house-project\src\utils to D:\g-house-project\frontend_H_book\src\utils
- Verified all import paths are correct in the components

2. ADDING TRANSLATION BUTTON TO HERO SECTION:
- Updated HeroSection.jsx to import TranslationToggle component
- Added TranslationToggle component to the hero section of the homepage
- Positioned the component at the top-right of the hero section with proper styling
- Set initial state to English as requested

The Roman Urdu translation feature is now properly located in the Docusaurus project structure and the translation button has been added to the hero section of the homepage as requested.

## OUTCOME
- Files moved to correct location (frontend_H_book/src/)
- Translation button added to homepage hero section
- All functionality preserved
- Import paths verified and working correctly