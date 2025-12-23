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

export default function ModuleCards() {
  return (
    <section className={styles.features} aria-labelledby="modules-heading">
      <div className="container">
        <h2 id="modules-heading" style={{textAlign: 'center', marginBottom: '3rem', color: 'var(--ifm-color-primary-darkest)', width: '100%'}}>
          A structured journey from robotic foundations to autonomous humanoids.
        </h2>
        <div className="row">
          {ModuleList.map((props, idx) => (
            <div className="col col--3" key={idx}>
              <div className="text--center padding-horiz--md">
                <h3 style={{color: 'var(--ifm-color-primary-darkest)', fontSize: '1.3rem', marginBottom: '1rem'}}>
                  {props.title}
                </h3>
                <p>{props.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}