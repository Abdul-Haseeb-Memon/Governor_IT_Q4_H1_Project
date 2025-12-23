---
id: 001
title: Implementation Plan
stage: plan
date: 2025-12-22
surface: agent
model: Claude Sonnet 4.5
feature: urdu-translation-fix
branch: 006-urdu-translation-fix
user: Claude Code
command: /sp.plan
labels: ["translation", "urdu", "navbar", "docusaurus", "plan"]
links:
  spec: specs/006-urdu-translation-fix/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/006-urdu-translation-fix/plan.md
 - specs/006-urdu-translation-fix/research.md
 - specs/006-urdu-translation-fix/data-model.md
 - specs/006-urdu-translation-fix/quickstart.md
 - specs/006-urdu-translation-fix/contracts/translation-api.yaml
 - CLAUDE.md
tests:
 - none
---

## Prompt

```
/sp.plan do your word and this addiontAL
Plan:
1. Identify and deprecate all previous translation-related specs (spec-005, spec-001-site-urdu), keeping this plan as the source of truth.
2. Remove duplicate or partial translation logic and merge into a single, sentence-level translation implementation.
3. Move the translation toggle to the top navbar beside the GitHub link and light/dark mode toggle, matching the same theme style.
4. Apply translation site-wide (homepage, docs, sidebar) while excluding code blocks and technical literals.
5. Validate changes by running `npm run build` and `npm run start`, ensuring no errors and stable GitHub Pages deployment.
```

## Response snapshot

Successfully completed the implementation planning for the English to Urdu-Pakistan translation with proper navbar integration. The plan includes:

1. **Comprehensive Implementation Plan** (plan.md): Detailed architecture for replacing existing partial/duplicate translation implementations with a single, sentence-level translation system positioned in the top navbar.

2. **Research Documentation** (research.md): Analysis of translation methods, DOM manipulation strategies, and best practices for Docusaurus integration with focus on complete word translation.

3. **Data Model** (data-model.md): Defined entities for TranslationState, TranslationDictionary, TranslationRequest/Response, and DOMElementSelector with proper relationships and validation.

4. **API Contracts** (contracts/translation-api.yaml): OpenAPI specification for the translation interface including state management and translation operations.

5. **Quickstart Guide** (quickstart.md): Step-by-step instructions for implementing the translation feature with proper navbar integration.

6. **Agent Context Update**: Updated CLAUDE.md with new technology stack information (JavaScript ES6+, React 18, Docusaurus v3+, etc.).

The plan addresses all user requirements:
- Deprecates previous translation implementations (spec-005, spec-001-site-urdu)
- Merges into single sentence-level translation implementation
- Positions toggle in top navbar beside GitHub link and theme toggle
- Applies translation site-wide while excluding code blocks
- Maintains GitHub Pages compatibility

## Outcome

- ✅ Impact: Comprehensive plan for Urdu-Pakistan translation with proper navbar integration
- 🧪 Tests: No automated tests for planning phase
- 📁 Files: plan.md, research.md, data-model.md, quickstart.md, contracts, CLAUDE.md updated
- 🔁 Next prompts: Ready for /sp.tasks to generate implementation tasks
- 🧠 Reflection: Plan consolidates previous implementations and addresses all user concerns

## Evaluation notes (flywheel)

- Failure modes observed: None - all requirements addressed in plan
- Graders run and results (PASS/FAIL): All constitution principles validated
- Prompt variant (if applicable): Planning with additional requirements
- Next experiment (smallest change to try): Task generation for implementation