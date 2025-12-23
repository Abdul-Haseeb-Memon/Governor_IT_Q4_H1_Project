# Quickstart: Book Homepage Implementation

## Overview
Quick guide to implement the professional homepage for "Physical AI & Humanoid Robotics" book using Docusaurus.

## Prerequisites
- Node.js 18+ installed
- Docusaurus CLI installed (`npm install -g @docusaurus/cli`)
- Basic knowledge of React and MDX

## Setup Steps

### 1. Navigate to the Docusaurus Project
```bash
cd frontend_H_book
```

### 2. Create the Homepage Component
Create the main homepage file:
```bash
mkdir -p src/pages
touch src/pages/index.js
```

### 3. Create Custom Components Directory
```bash
mkdir -p src/components/Homepage
touch src/components/Homepage/HeroSection.jsx
touch src/components/Homepage/ModuleCards.jsx
```

### 4. Create Custom CSS
```bash
mkdir -p src/css
touch src/css/custom.css
```

## Implementation Steps

### Step 1: Create the Main Homepage File
Edit `src/pages/index.js`:

```jsx
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/Homepage/HeroSection';
import styles from './index.module.css';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Homepage`}
      description="Professional homepage for Physical AI & Humanoid Robotics book">
      <HomepageFeatures />
    </Layout>
  );
}
```

### Step 2: Create Hero Section Component
Edit `src/components/Homepage/HeroSection.jsx`:

```jsx
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './HeroSection.module.css';

export default function HomepageHero() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">Physical AI & Humanoid Robotics</h1>
        <p className="hero__subtitle">Bridging AI Intelligence with Real-World Humanoid Robotics</p>
        <p className="hero__description">
          Explore how artificial intelligence moves beyond screens into physical machines. Learn to design, simulate, and control humanoid robots using modern AI and robotics frameworks.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Read the Book
          </Link>
        </div>
      </div>
    </header>
  );
}
```

### Step 3: Create Module Cards Component
Edit `src/components/Homepage/ModuleCards.jsx`:

```jsx
import React from 'react';
import clsx from 'clsx';
import styles from './ModuleCards.module.css';

const ModuleList = [
  {
    title: 'The Robotic Nervous System (ROS 2)',
    description: 'Middleware, nodes, topics, services, Python agents, URDF',
  },
  {
    title: 'The Digital Twin (Gazebo & Unity)',
    description: 'Physics simulation, environments, sensors, realism',
  },
  {
    title: 'The AI-Robot Brain (NVIDIA Isaac™)',
    description: 'Perception, VSLAM, navigation, synthetic data',
  },
  {
    title: 'Vision-Language-Action (VLA)',
    description: 'Voice commands, LLM planning, autonomous humanoids',
  },
];

function ModuleCard({title, description}) {
  return (
    <div className={clsx('col col--3')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function ModuleCards() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <h2>A structured journey from robotic foundations to autonomous humanoids.</h2>
          {ModuleList.map((props, idx) => (
            <ModuleCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
```

### Step 4: Add Custom Styling
Edit `src/css/custom.css` with the deep blue/steel blue color scheme:

```css
/* Deep blue/steel blue color scheme */
:root {
  --ifm-color-primary: #1e40af; /* Deep blue */
  --ifm-color-primary-dark: #1d4ed8;
  --ifm-color-primary-darker: #1e40af;
  --ifm-color-primary-darkest: #1e3a8a;
  --ifm-color-primary-light: #3b82f6; /* Steel blue */
  --ifm-color-primary-lighter: #60a5fa;
  --ifm-color-primary-lightest: #93c5fd;
  --ifm-code-font-size: 95%;
}

/* Custom homepage styles */
.hero--primary {
  background: linear-gradient(135deg, var(--ifm-color-primary-darkest) 0%, var(--ifm-color-primary-dark) 100%);
  color: white;
}

.hero__title {
  color: white;
  font-size: 3rem;
  font-weight: bold;
  margin-bottom: 1rem;
}

.hero__subtitle {
  color: var(--ifm-color-primary-lighter);
  font-size: 1.5rem;
  margin-bottom: 1rem;
}

.hero__description {
  color: white;
  font-size: 1.2rem;
  margin-bottom: 2rem;
  max-width: 800px;
  margin-left: auto;
  margin-right: auto;
}

/* Module cards styling */
.features {
  padding: 4rem 0;
  background-color: #f9fafb;
}

.features h2 {
  text-align: center;
  margin-bottom: 3rem;
  color: var(--ifm-color-primary-darkest);
}

/* Card styling */
.col {
  margin-bottom: 2rem;
}

.text--center h3 {
  color: var(--ifm-color-primary-darkest);
  font-size: 1.3rem;
  margin-bottom: 1rem;
}

/* Responsive adjustments */
@media (max-width: 996px) {
  .hero__title {
    font-size: 2.5rem;
  }

  .hero__subtitle {
    font-size: 1.2rem;
  }

  .hero__description {
    font-size: 1rem;
  }
}
```

### Step 5: Update Docusaurus Configuration
Ensure the homepage is set as the default route by checking `docusaurus.config.js`:

```js
// Make sure the presets include the classic preset
presets: [
  [
    'classic',
    /** @type {import('@docusaurus/preset-classic').Options} */
    ({
      docs: {
        sidebarPath: require.resolve('./sidebars.js'),
      },
      theme: {
        customCss: require.resolve('./src/css/custom.css'),
      },
    }),
  ],
],
```

## Build and Test

### 1. Install Dependencies
```bash
npm install
```

### 2. Start Development Server
```bash
npm start
```

### 3. Build for Production
```bash
npm run build
```

## Deployment
The built site can be deployed to GitHub Pages by pushing to the appropriate branch.

## Customization
- Modify colors in `src/css/custom.css` to adjust the theme
- Update module content in `ModuleCards.jsx` to change descriptions
- Adjust layout in `HeroSection.jsx` to modify the header section