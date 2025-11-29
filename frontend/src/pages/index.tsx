import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  const heroImage = useBaseUrl('/img/hero-robot.svg');
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
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
          <div className={styles.heroImage}>
            <img
              src={heroImage}
              alt="Humanoid robot with AI neural network connections representing Physical AI"
              width="280"
              height="280"
            />
          </div>
        </div>
      </div>
    </header>
  );
}

interface ModuleInfo {
  title: string;
  description: string;
  link: string;
  icon: string;
}

function ModuleCard({module}: {module: ModuleInfo}): ReactNode {
  const iconUrl = useBaseUrl(module.icon);
  return (
    <div className="col col--6 margin-bottom--lg">
      <div className={styles.moduleCard}>
        <div className={styles.moduleIcon}>
          <img src={iconUrl} alt={module.title} width="64" height="64" />
        </div>
        <Heading as="h3">{module.title}</Heading>
        <p>{module.description}</p>
        <Link className="button button--primary button--sm" to={module.link}>
          Explore Module
        </Link>
      </div>
    </div>
  );
}

function LearningJourney(): ReactNode {
  const journeyGif = useBaseUrl('/img/animations/build-robot-journey.gif');
  return (
    <section className={styles.learningJourney}>
      <div className="container">
        <Heading as="h2" className="text--center margin-bottom--lg">
          From Learning to Creation
        </Heading>
        <div className={styles.journeyAnimation}>
          <img
            src={journeyGif}
            alt="Animation showing the complete journey: A human learns from the Physical AI book, gains knowledge of ROS2, Gazebo, Isaac, and VLA, then builds and assembles a humanoid robot which comes to life and works autonomously"
            className={styles.journeyGif}
          />
        </div>
        <p className="text--center margin-top--md">
          Learn Physical AI concepts, build your own humanoid robot, and bring it to life!
        </p>
      </div>
    </section>
  );
}

function ModuleCards(): ReactNode {
  const modules: ModuleInfo[] = [
    {
      title: 'Module 1: ROS2 Fundamentals',
      description: 'Master the Robot Operating System 2 - nodes, topics, services, actions, and Python integration with rclpy.',
      link: '/docs/module-1-ros2',
      icon: '/img/icons/ros2-icon.svg',
    },
    {
      title: 'Module 2: Robot Simulation',
      description: 'Learn Gazebo simulation, physics engines, sensor modeling, and URDF robot descriptions.',
      link: '/docs/module-2-simulation',
      icon: '/img/icons/simulation-icon.svg',
    },
    {
      title: 'Module 3: NVIDIA Isaac',
      description: 'GPU-accelerated robotics with Isaac Sim, Isaac ROS, photorealistic simulation, and edge deployment.',
      link: '/docs/module-3-nvidia-isaac',
      icon: '/img/icons/isaac-icon.svg',
    },
    {
      title: 'Module 4: VLA Models',
      description: 'Vision-Language-Action models, voice commands, LLM integration, and natural language robot control.',
      link: '/docs/module-4-vla',
      icon: '/img/icons/vla-icon.svg',
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
            <ModuleCard key={idx} module={module} />
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
        <LearningJourney />
        <ModuleCards />
      </main>
    </Layout>
  );
}
