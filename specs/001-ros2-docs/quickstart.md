# Quickstart: ROS 2 Documentation Module

**Feature**: 001-ros2-docs
**Date**: 2025-12-17

## Prerequisites

- Node.js 18+ installed
- npm or yarn package manager
- Basic knowledge of Markdown syntax
- Understanding of ROS 2 concepts (optional but helpful)

## Setup Instructions

1. **Initialize Docusaurus Project**
   ```bash
   npx create-docusaurus@latest website classic
   cd website
   ```

2. **Install Dependencies**
   ```bash
   npm install
   # or
   yarn install
   ```

3. **Create Module Directory**
   ```bash
   mkdir -p docs/module-1-ros2
   ```

4. **Add Module Content Files**
   Create the three chapter files:
   - `docs/module-1-ros2/01-ros2-fundamentals.md`
   - `docs/module-1-ros2/02-python-agents-rclpy.md`
   - `docs/module-1-ros2/03-humanoid-modeling-urdf.md`

5. **Update Sidebar Configuration**
   Add the module to `sidebars.js`:
   ```javascript
   module.exports = {
     tutorial: [
       // existing content...
       {
         type: 'category',
         label: 'Module 1: The Robotic Nervous System (ROS 2)',
         items: [
           'module-1-ros2/01-ros2-fundamentals',
           'module-1-ros2/02-python-agents-rclpy',
           'module-1-ros2/03-humanoid-modeling-urdf',
         ],
       },
     ],
   };
   ```

6. **Start Development Server**
   ```bash
   npm run start
   # or
   yarn start
   ```

## Content Creation Workflow

1. **Write Chapter Content**
   - Create each chapter as a Markdown file in `docs/module-1-ros2/`
   - Use Docusaurus Markdown features like admonitions, code blocks, and links
   - Include learning objectives at the beginning of each chapter

2. **Validate Content**
   - Run `npm run build` to ensure the site builds without errors
   - Check that all links work correctly
   - Verify that code examples are properly formatted

3. **Preview Changes**
   - Use the development server to preview changes in real-time
   - Test navigation between chapters

## Deployment

1. **Build Static Site**
   ```bash
   npm run build
   ```

2. **Deploy to GitHub Pages**
   ```bash
   npm run deploy
   ```

## Troubleshooting

- **Build errors**: Check Markdown syntax and ensure all referenced files exist
- **Sidebar not showing**: Verify that file paths in `sidebars.js` match actual file locations
- **Images not loading**: Place images in `static/img/` and reference with `/img/path-to-image`
- **Links not working**: Use relative paths within the docs directory or absolute paths for external links