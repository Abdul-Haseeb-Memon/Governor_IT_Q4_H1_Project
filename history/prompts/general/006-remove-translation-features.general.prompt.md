---
id: 6
title: "Remove Translation Features"
stage: "general"
date: "2025-12-23"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "none"
branch: "001-fix-translation-all-modules"
user: "ahas3"
command: "/sp.implement read all the project and cut the addtional thing like <span class=\"translation-icon\" aria-hidden=\"true\">EN</span> this and all tyranlate feauture and make it only the specs that are present"
labels: ["cleanup", "translation", "docusaurus", "features"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "frontend_H_book/src/theme/Navbar/index.jsx"
  - "frontend_H_book/src/contexts/TranslationContext.jsx"
  - "frontend_H_book/src/components/TranslationToggle/"
  - "frontend_H_book/src/theme/Root.jsx"
  - "frontend_H_book/src/utils/domUtils.js"
  - "frontend_H_book/src/css/custom.css"
tests: []
---

## Prompt

```
read all the project and cut the addtional thing like <span class="translation-icon" aria-hidden="true">EN</span> this and all tyranlate feauture and make it only the specs that are present
```

## Response

Successfully completed the user's request to remove all translation features from the project:

1. Removed translation toggle from navbar in frontend_H_book/src/theme/Navbar/index.jsx
2. Removed TranslationContext.jsx file entirely
3. Removed TranslationToggle directory and all its components
4. Removed other translation-related files:
   - NavbarTranslationToggle.jsx
   - CustomTranslationToggle.jsx
   - NavbarItemCustomTranslationToggle.tsx
5. Updated Root.jsx to remove TranslationProvider wrapper
6. Updated domUtils.js to remove translation-specific functions while keeping general DOM utilities
7. Removed translation-related CSS from custom.css
8. Verified that the website still builds successfully without translation features

The website now functions without any translation features, keeping only the core documentation functionality and the existing specifications.

## Outcome

Successfully removed all translation features from the project while maintaining the core functionality. The website builds and runs properly without the translation system.