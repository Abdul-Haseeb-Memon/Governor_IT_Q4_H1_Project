<!--
Sync Impact Report:
- Version change: N/A → 1.0.0 (initial constitution)
- Modified principles: N/A (new principles created)
- Added sections: Core Principles, Book Standards, AI Authoring Rules, RAG Chatbot Requirements, Constraints
- Removed sections: None (all template placeholders filled)
- Templates requiring updates: N/A (initial creation)
- Follow-up TODOs: None
-->
# AI-Spec-Driven Technical Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-First, AI-Driven Authoring
All content creation follows the Spec-Kit Plus methodology with Claude Code implementation. Technical book chapters and features must originate from formal specifications before implementation begins. This ensures structured, consistent, and maintainable content production.

### Technical Accuracy and Clarity
Every chapter, code example, and explanation must meet professional technical writing standards. Content must be factually accurate, clearly explained, and accessible to the target audience. All code examples must be runnable and thoroughly tested before publication.

### Reproducibility and Maintainability
The book building process, deployment pipeline, and RAG chatbot infrastructure must be reproducible by any team member. All processes should be documented, automated where possible, and designed for long-term maintenance with minimal technical debt.

### No Unsupported or Speculative Content
All content must be based on proven technologies, established practices, or formally specified features. No experimental, unverified, or hypothetical content should be included without clear disclaimers and proper categorization.

### Docusaurus-First Documentation Framework
The technical book platform must leverage Docusaurus with MDX for content delivery. This ensures modern, responsive documentation with built-in features like search, versioning, and theming capabilities.

### RAG-Powered Chatbot Integration
The embedded chatbot must provide accurate, context-aware responses based solely on book content. The system must prevent hallucinations and maintain strict context boundaries to ensure reliable user assistance.

## Book Standards

- **Format**: Docusaurus with MDX for maximum flexibility and interactivity
- **Structure**: Clear chapter organization with logical navigation flow
- **Code Examples**: Runnable, well-documented, and accompanied by explanations
- **Tone**: Consistent professional technical writing suitable for the target audience
- **Navigation**: Intuitive sidebar, breadcrumbs, and cross-references between sections

## AI Authoring Rules

- **Spec Compliance**: All content must derive directly from Spec-Kit Plus specifications
- **Claude Code Implementation**: Claude Code must implement specifications into chapters following defined patterns
- **Specification Adherence**: Strict adherence to defined specifications without deviation or improvisation
- **Quality Assurance**: All AI-generated content requires human review for accuracy and clarity

## RAG Chatbot Requirements

- **Integration**: Embedded seamlessly within the Docusaurus site experience
- **Technology Stack**: OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, Qdrant Cloud (Free Tier)
- **Capabilities**:
  - Answer questions about the full book content corpus
  - Respond to queries based on user-selected text snippets
  - Maintain context-bound responses without hallucinations
- **Performance**: Fast response times with reliable availability
- **Security**: Secure handling of user queries and system configurations

## Constraints

- **Deployment**: GitHub Pages hosting for cost-effective and reliable distribution
- **Infrastructure**: Free-tier compatible services to maintain budget constraints
- **Security**: Proper handling of API keys, secrets, and configuration files
- **Scalability**: Architecture must accommodate growth within free-tier limitations

## Development Workflow

- **Spec-First Process**: All features begin with specification creation in Spec-Kit Plus
- **Implementation Pipeline**: Specifications → Claude Code implementation → Review → Deployment
- **Quality Gates**: All content and code must pass accuracy, clarity, and functionality checks
- **Version Control**: Proper Git workflow with meaningful commit messages and PR reviews
- **Testing**: Code examples must be verified as runnable; chatbot responses must be validated for accuracy

## Governance

This constitution serves as the authoritative guide for all project decisions. All team members must comply with these principles. Amendments require formal documentation, team approval, and migration planning. All pull requests and reviews must verify constitutional compliance. Complexity must be justified with clear benefits to the project goals.

**Version**: 1.0.0 | **Ratified**: 2025-12-17 | **Last Amended**: 2025-12-17