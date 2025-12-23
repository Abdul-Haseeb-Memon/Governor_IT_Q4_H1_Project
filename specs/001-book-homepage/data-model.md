# Data Model: Book Homepage (Docusaurus Root)

## Overview
Data structures for the book homepage content, focusing on the presentation of book modules and homepage elements.

## Homepage Content Structure

### HomepageData
- **title**: string (required) - The main title of the homepage
  - Example: "Physical AI & Humanoid Robotics"
  - Validation: Non-empty string, max 100 characters
- **subtitle**: string (required) - Subtitle for the hero section
  - Example: "Bridging AI Intelligence with Real-World Humanoid Robotics"
  - Validation: Non-empty string, max 150 characters
- **description**: string (required) - Main description text
  - Example: "Explore how artificial intelligence moves beyond screens into physical machines..."
  - Validation: Non-empty string, max 500 characters
- **ctaText**: string (required) - Call-to-action button text
  - Example: "Read the Book"
  - Validation: Non-empty string, max 50 characters
- **ctaLink**: string (required) - Destination URL for the CTA
  - Example: "/docs/intro"
  - Validation: Valid URL path
- **moduleSections**: array[ModuleCard] (required) - List of module cards
  - Validation: Exactly 4 modules in specified order

### ModuleCard
- **id**: string (required) - Unique identifier for the module
  - Example: "robotic-nervous-system"
  - Validation: URL-safe string, unique within homepage
- **title**: string (required) - Title of the module
  - Example: "The Robotic Nervous System (ROS 2)"
  - Validation: Non-empty string, max 100 characters
- **description**: string (required) - Brief description of module content
  - Example: "Middleware, nodes, topics, services, Python agents, URDF"
  - Validation: Non-empty string, max 200 characters
- **order**: number (required) - Display order (1-4)
  - Example: 1
  - Validation: Integer between 1 and 4, unique within homepage

## Color Scheme Data

### ColorScheme
- **primaryColor**: string (required) - Primary brand color
  - Example: "#1E3A8A" (deep blue)
  - Validation: Valid hex color code
- **secondaryColor**: string (required) - Secondary accent color
  - Example: "#3B82F6" (steel blue)
  - Validation: Valid hex color code
- **textColor**: string (required) - Primary text color
  - Example: "#FFFFFF" (white)
  - Validation: Valid hex color code
- **backgroundColor**: string (required) - Background color
  - Example: "#F9FAFB" (light background)
  - Validation: Valid hex color code

## Component States

### HomepageState
- **isLoading**: boolean (default: false) - Whether content is loading
- **error**: string (optional) - Error message if loading fails
- **displayMode**: string (default: "desktop") - Current responsive mode
  - Values: "mobile" | "tablet" | "desktop"

## Relationships
- Each ModuleCard belongs to exactly one HomepageData
- The HomepageData contains a collection of 4 ModuleCard items in a specific order
- ColorScheme is applied to the overall HomepageData presentation

## Validation Rules
- Homepage must have exactly 4 ModuleCard items
- ModuleCard items must be in the specified order: ROS 2, Digital Twin, AI-Robot Brain, VLA
- All required fields must be present and non-empty
- ModuleCard titles and descriptions must match the specifications exactly
- CTA link must be a valid internal or external URL