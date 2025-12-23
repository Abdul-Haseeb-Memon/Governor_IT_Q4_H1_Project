import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageHero from '@site/src/components/Homepage/HeroSection';
import ModuleCards from '@site/src/components/Homepage/ModuleCards';
import styles from './index.module.css';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="Professional homepage for Physical AI & Humanoid Robotics book - Learn about AI, robotics, ROS 2, simulation, and autonomous humanoids">
      <HomepageHero />
      <ModuleCards />
    </Layout>
  );
}