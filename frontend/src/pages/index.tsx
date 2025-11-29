import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning 🚀
          </Link>
        </div>
      </div>
    </header>
  );
}

function ModuleCards(): ReactNode {
  const modules = [
    {
      title: 'Module 1: ROS2 Fundamentals',
      description: 'Master the Robot Operating System 2 - nodes, topics, services, actions, and Python integration with rclpy.',
      link: '/docs/module-1-ros2',
      emoji: '🤖',
    },
    {
      title: 'Module 2: Robot Simulation',
      description: 'Learn Gazebo simulation, physics engines, sensor modeling, and URDF robot descriptions.',
      link: '/docs/module-2-simulation',
      emoji: '🎮',
    },
    {
      title: 'Module 3: NVIDIA Isaac',
      description: 'GPU-accelerated robotics with Isaac Sim, Isaac ROS, photorealistic simulation, and edge deployment.',
      link: '/docs/module-3-nvidia-isaac',
      emoji: '⚡',
    },
    {
      title: 'Module 4: VLA Models',
      description: 'Vision-Language-Action models, voice commands, LLM integration, and natural language robot control.',
      link: '/docs/module-4-vla',
      emoji: '🧠',
    },
  ];

  return (
    <section className={styles.modules}>
      <div className="container">
        <Heading as="h2" className="text--center margin-bottom--lg">
          Course Modules
        </Heading>
        <div className="row">
          {modules.map((module, idx) => (
            <div key={idx} className="col col--6 margin-bottom--lg">
              <div className={styles.moduleCard}>
                <div className={styles.moduleEmoji}>{module.emoji}</div>
                <Heading as="h3">{module.title}</Heading>
                <p>{module.description}</p>
                <Link className="button button--primary button--sm" to={module.link}>
                  Explore Module →
                </Link>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Learn to build intelligent humanoid robots with ROS2, Gazebo, NVIDIA Isaac, and Vision-Language-Action models">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <ModuleCards />
      </main>
    </Layout>
  );
}
