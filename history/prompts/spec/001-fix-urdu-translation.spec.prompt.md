---
id: 001
title: Fix Urdu Translation
stage: spec
date: 2025-12-22
surface: agent
model: Claude Sonnet 4.5
feature: urdu-translation-fix
branch: 006-urdu-translation-fix
user: Claude Code
command: /sp.specify
labels: ["translation", "urdu", "navbar", "docusaurus"]
links:
  spec: specs/006-urdu-translation-fix/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/006-urdu-translation-fix/spec.md
 - specs/006-urdu-translation-fix/checklists/requirements.md
tests:
 - none
---

## Prompt

```
/sp.specify their is some mistake in my code i want th translation in english to urdu-pakistan and i make two spesify 005 and i went wrong so i make the other spec 001-site-urdu and it work but is compler messs i convet only half owrd so and the botten is not corect loaction or in nevbar so i have to be in navebar where the github link and light mode are and same theme so reade my project and read all thing and do corect
```

## Response snapshot

Created a comprehensive specification for fixing the Urdu translation functionality based on user requirements. The specification addresses:

1. Proper navbar integration with translation toggle positioned next to GitHub link and theme toggle
2. Complete word translation instead of partial word conversion
3. Matching theme and styling with existing navbar elements
4. RTL (right-to-left) layout support for Urdu-Pakistan
5. Translation state persistence across navigation

The specification includes three prioritized user stories:
- User Story 1 (P1): Navbar Translation Toggle - core functionality with proper positioning
- User Story 2 (P1): Complete Word Translation - addressing the partial word issue
- User Story 3 (P2): Proper Navbar Integration - ensuring consistent styling and placement

All requirements are testable and measurable with clear acceptance scenarios. The specification is ready for the planning phase and includes all necessary components for implementation.

## Outcome

- ✅ Impact: Comprehensive specification for Urdu-Pakistan translation fix
- 🧪 Tests: No automated tests for specification phase
- 📁 Files: spec.md and requirements checklist created
- 🔁 Next prompts: Ready for /sp.plan to create implementation plan
- 🧠 Reflection: Specification addresses all user concerns about translation quality and navbar placement

## Evaluation notes (flywheel)

- Failure modes observed: None - all requirements clearly defined
- Graders run and results (PASS/FAIL): All checklist items PASS
- Prompt variant (if applicable): Specification creation for translation fix
- Next experiment (smallest change to try): Planning the implementation