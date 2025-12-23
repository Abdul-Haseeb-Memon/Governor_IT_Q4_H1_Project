# Research: Book Homepage (Docusaurus Root)

## Overview
Research for implementing the professional homepage for "Physical AI & Humanoid Robotics" book using Docusaurus with MDX structure.

## Decision: Homepage Structure
**Rationale**: Using Docusaurus with a custom index.js page allows for complete control over the homepage design while maintaining compatibility with the documentation framework. This approach follows Docusaurus best practices for custom homepages.

**Alternatives considered**:
- Using Docusaurus preset themes: Limited customization options that don't meet the academic/professional design requirements
- Separate React app: Would complicate deployment and maintenance without significant benefits

## Decision: MDX Implementation
**Rationale**: MDX allows for embedding React components within Markdown, providing the flexibility needed for the card-based layout and custom components while maintaining content readability.

**Alternatives considered**:
- Pure React components: More complex to maintain content
- Standard HTML/CSS: Would lose Docusaurus integration benefits

## Decision: Color Scheme and Styling
**Rationale**: Deep blue/steel blue base with white text and soft gradients creates a professional, academic appearance that aligns with the "serious university-level textbook" requirement.

**Alternatives considered**:
- Corporate color schemes: Might appear too commercial rather than academic
- Colorful/gradient-heavy designs: Would conflict with the "no marketing fluff" requirement

## Decision: Responsive Design Approach
**Rationale**: Mobile-first responsive design ensures the homepage works well across all device sizes, meeting the 95% display requirement from the spec.

**Alternatives considered**:
- Desktop-only design: Would exclude mobile users
- Separate mobile site: Would increase complexity without significant benefit

## Decision: Card-Based Layout for Modules
**Rationale**: Card layout provides clear visual separation of the four book modules while maintaining a clean, organized appearance that fits the academic style.

**Alternatives considered**:
- List format: Would be less visually appealing and harder to scan
- Grid layout: Could become cluttered with 4 modules

## Docusaurus Best Practices Applied
- Custom homepage using src/pages/index.js
- Component-based architecture for maintainability
- CSS modules for scoped styling
- Semantic HTML for accessibility
- Performance optimization for fast loading