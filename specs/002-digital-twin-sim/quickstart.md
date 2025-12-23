# Quickstart: Digital Twin Simulation Documentation

**Feature**: 002-digital-twin-sim
**Date**: 2025-12-18

## Prerequisites

- Basic knowledge of ROS 2 concepts (covered in Module 1)
- Understanding of fundamental robotics concepts
- Familiarity with command-line interfaces
- Basic understanding of 3D environments (helpful but not required)

## Setup Instructions

1. **Ensure Module 1 is Complete**
   ```bash
   # Verify Module 1 content is accessible
   cd frontend_H_book
   ls docs/module-1-ros2/
   ```

2. **Create Module Directory**
   ```bash
   mkdir -p docs/module-2-digital-twin
   ```

3. **Add Module Content Files**
   Create the three chapter files:
   - `docs/module-2-digital-twin/01-physics-simulation-gazebo.md`
   - `docs/module-2-digital-twin/02-high-fidelity-unity.md`
   - `docs/module-2-digital-twin/03-sensor-simulation.md`

4. **Update Sidebar Configuration**
   Add the module to `sidebars.js`:
   ```javascript
   module.exports = {
     tutorial: [
       // existing content...
       {
         type: 'category',
         label: 'Module 2: The Digital Twin (Gazebo & Unity)',
         items: [
           'module-2-digital-twin/01-physics-simulation-gazebo',
           'module-2-digital-twin/02-high-fidelity-unity',
           'module-2-digital-twin/03-sensor-simulation',
         ],
       },
       // other modules...
     ],
   };
   ```

5. **Start Development Server**
   ```bash
   npm run start
   ```

## Content Creation Workflow

1. **Write Chapter Content**
   - Create each chapter as a Markdown file in `docs/module-2-digital-twin/`
   - Use Docusaurus Markdown features like admonitions, code blocks, and links
   - Include learning objectives at the beginning of each chapter

2. **Validate Content**
   - Run `npm run build` to ensure the site builds without errors
   - Check that all links work correctly
   - Verify that code examples are properly formatted

3. **Preview Changes**
   - Use the development server to preview changes in real-time
   - Test navigation between chapters
   - Verify that simulation concepts are clearly explained

## Chapter-Specific Setup

### For Gazebo Physics Simulation:
- Ensure Gazebo Classic or Garden/Harmonic is available for reference
- Have example robot models ready for demonstrations
- Prepare physics parameter examples

### For Unity High-Fidelity Environments:
- Document Unity LTS version requirements
- Prepare visual asset examples
- Explain Unity-ROS 2 integration patterns

### For Sensor Simulation:
- Have ROS 2 sensor message types reference available
- Prepare sensor configuration examples
- Document sensor data validation techniques

## Troubleshooting

- **Build errors**: Check Markdown syntax and ensure all referenced files exist
- **Sidebar not showing**: Verify that file paths in `sidebars.js` match actual file locations
- **Images not loading**: Place images in `static/img/` and reference with `/img/path-to-image`
- **Links not working**: Use relative paths within the docs directory or absolute paths for external links
- **Cross-module references**: Use full paths when referencing content from other modules