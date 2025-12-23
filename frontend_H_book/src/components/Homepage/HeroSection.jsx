import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function HomepageHero() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <header className={clsx('hero hero--primary')}>
      <div className="container">
        <h1 className="hero__title">Physical AI & Humanoid Robotics</h1>
        <p className="hero__subtitle">Bridging AI Intelligence with Real-World Humanoid Robotics</p>
        <p className="hero__description">
          Explore how artificial intelligence moves beyond screens into physical machines. Learn to design, simulate, and control humanoid robots using modern AI and robotics frameworks.
        </p>
        <div className="button-container">
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro"
            aria-label="Read the book - start learning about Physical AI & Humanoid Robotics">
            Read the Book
          </Link>
        </div>
      </div>
    </header>
  );
}