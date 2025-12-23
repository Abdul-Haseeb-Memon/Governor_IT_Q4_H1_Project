---
id: 001
title: Implementation Plan
stage: plan
date: 2025-12-22
surface: agent
model: Claude Sonnet 4.5
feature: site-urdu-translation-authoritative
branch: 007-site-urdu-translation-authoritative
user: Claude Code
command: /sp.plan
labels: ["translation", "urdu", "navbar", "docusaurus", "plan"]
links:
  spec: specs/007-site-urdu-translation-authoritative/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/007-site-urdu-translation-authoritative/plan.md
 - specs/007-site-urdu-translation-authoritative/research.md
 - specs/007-site-urdu-translation-authoritative/data-model.md
 - specs/007-site-urdu-translation-authoritative/quickstart.md
 - specs/007-site-urdu-translation-authoritative/contracts/translation-api.yaml
 - CLAUDE.md
tests:
 - none
---

## Prompt

```
/sp.plan
ID: 007-site-urdu-translation-authoritative

Plan:
1. Deprecate all previous translation specs (001-site-urdu-translation, 005-translate-roman-urdu, 006-urdu-translation-fix) and enforce this spec as the single source of truth.
2. Move the language toggle to the top navbar beside the GitHub link and theme switch, and implement full site-wide English → Urdu (Pakistan) translation.
3. Validate by running `npm run build` and `npm run start`, ensuring no partial translations or UI duplication.
```

## Response snapshot

Successfully completed the implementation planning for the authoritative English to Urdu (Pakistan) translation feature. The plan addresses all requirements from the user input:

1. **Deprecation of Previous Specs**: The plan enforces this spec (007) as the single source of truth, superseding and deprecating all previous translation specifications (001, 005, 006).

2. **Navbar Integration**: The implementation plan specifies positioning the translation toggle in the top navbar beside the GitHub link and theme switch with matching styling, as required.

3. **Full Site-Wide Translation**: The plan covers complete English → Urdu (Pakistan) translation across the entire site (homepage, modules, chapters, sidebar, navigation labels) with sentence-level processing to avoid partial translations.

4. **Technical Implementation**: The plan includes:
   - Sentence-level translation with complete word processing (no partial or word-level translation)
   - Urdu (Pakistan) only (no Roman Urdu mixed behavior)
   - Code preservation during translation
   - RTL layout support for Urdu
   - Performance optimization (2-second requirement)
   - Proper navbar positioning with GitHub link and theme toggle alignment

5. **Validation Requirements**: The plan ensures build validation (`npm run build`) and development server validation (`npm run start`) with no UI duplication or partial translation artifacts.

The planning phase generated all required artifacts:
- Implementation Plan (plan.md) with technical context and constitution compliance
- Research documentation (research.md) with technical decisions and alternatives
- Data model (data-model.md) with entity definitions
- API contracts (contracts/translation-api.yaml) with interface specifications
- Quickstart guide (quickstart.md) with installation instructions
- Agent context updated (CLAUDE.md) with new technology stack information

All constitution principles have been validated and the implementation is ready for the next phase.

## Outcome

- ✅ Impact: Authoritative plan for Urdu translation feature that supersedes all previous specs
- 🧪 Tests: No automated tests for planning phase
- 📁 Files: plan.md, research.md, data-model.md, quickstart.md, contracts, CLAUDE.md updated
- 🔁 Next prompts: Ready for /sp.tasks to generate implementation tasks
- 🧠 Reflection: Plan addresses all issues identified in previous implementations

## Evaluation notes (flywheel)

- Failure modes observed: None - all requirements addressed in plan
- Graders run and results (PASS/FAIL): All constitution principles validated
- Prompt variant (if applicable): Planning with authoritative override requirements
- Next experiment (smallest change to try): Task generation for implementation