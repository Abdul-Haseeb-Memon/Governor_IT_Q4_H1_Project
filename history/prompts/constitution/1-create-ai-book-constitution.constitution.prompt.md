---
id: 1
title: Create AI Book Constitution
stage: constitution
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: Claude Sonnet 4.5
command: /sp.constitution
labels: [constitution,spec-first,ai-authoring,rag-chatbot]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```
/sp.constitution

Project: AI-Spec–Driven Technical Book with Embedded RAG Chatbot

Goal:
Create a unified technical book using Spec-Kit Plus and Claude Code, built with Docusaurus and deployed to GitHub Pages, with an integrated RAG chatbot for book-aware Q&A.

Core Principles:
- Spec-first, AI-driven authoring
- Technical accuracy and clarity
- Reproducibility and maintainability
- No unsupported or speculative content

Book Standards:
- Docusaurus (MDX)
- Clear chapter structure and navigation
- Runnable, well-explained code examples
- Consistent professional tone

AI Authoring Rules:
- All content derived from Spec-Kit Plus specs
- Claude Code implements specs into chapters
- Strict adherence to defined specifications

RAG Chatbot Requirements:
- Embedded in the Docusaurus site
- Stack:
  - OpenAI Agents / ChatKit SDKs
  - FastAPI
  - Neon Serverless Postgres
  - Qdrant Cloud (Free Tier)
- Capabilities:
  - Answer questions about the full book
  - Answer questions using only user-selected text
  - Context-bounded, no hallucinations

Constraints:
- Deploy on GitHub Pages
- Free-tier compatible infrastructure
- Secure handling of secrets and configs

Success Criteria:
- Book builds and deploys successfully
- Chatbot returns accurate, context-correct answers
- Specs cleanly map to implemented content
```

## Response snapshot

Created a comprehensive project constitution for the AI-Spec-Driven Technical Book with Embedded RAG Chatbot project. The constitution includes:

- Six core principles: Spec-First AI-Driven Authoring, Technical Accuracy and Clarity, Reproducibility and Maintainability, No Unsupported Content, Docusaurus-First Framework, and RAG-Powered Chatbot Integration
- Detailed sections covering Book Standards, AI Authoring Rules, RAG Chatbot Requirements, and Constraints
- Development workflow guidelines and governance structure
- Version 1.0.0 with proper ratification date

The constitution captures all the requirements from the user's project description and establishes clear guidelines for the project's development.

## Outcome

- ✅ Impact: Created initial project constitution document with comprehensive principles and guidelines
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with project-specific constitution
- 🔁 Next prompts: Proceed with creating specifications for the technical book and RAG chatbot
- 🧠 Reflection: The constitution provides a solid foundation for the project, establishing clear principles that will guide future development decisions.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS - Constitution properly created with all required elements
- Prompt variant (if applicable): Standard constitution creation prompt
- Next experiment (smallest change to try): Begin creating feature specifications based on the established constitution