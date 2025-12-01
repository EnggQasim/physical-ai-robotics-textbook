import type {ReactNode} from 'react';
import {useEffect, useRef, useState} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Layout from '@theme/Layout';
import Translate, {translate} from '@docusaurus/Translate';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import * as THREE from 'three';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  const heroImage = useBaseUrl('/img/hero-robot.svg');
  const vantaRef = useRef<HTMLDivElement>(null);
  const [vantaEffect, setVantaEffect] = useState<any>(null);

  useEffect(() => {
    // Dynamically import Vanta to avoid SSR issues
    const loadVanta = async () => {
      if (!vantaEffect && vantaRef.current) {
        try {
          const VANTA = await import('vanta/dist/vanta.net.min');
          const effect = VANTA.default({
            el: vantaRef.current,
            THREE: THREE,
            mouseControls: true,
            touchControls: true,
            gyroControls: false,
            minHeight: 200.0,
            minWidth: 200.0,
            scale: 1.0,
            scaleMobile: 1.0,
            color: 0x76b900,        // NVIDIA green for points
            backgroundColor: 0x1a4d00, // Darker green background
            points: 12.0,
            maxDistance: 22.0,
            spacing: 18.0,
            showDots: true,
          });
          setVantaEffect(effect);
        } catch (error) {
          console.error('Failed to load Vanta effect:', error);
        }
      }
    };
    loadVanta();

    return () => {
      if (vantaEffect) vantaEffect.destroy();
    };
  }, [vantaEffect]);

  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)} ref={vantaRef}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <Heading as="h1" className="hero__title">
              {siteConfig.title}
            </Heading>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <div className={styles.buttons}>
              <Link
                className={clsx("button button--secondary button--lg", styles.startButton)}
                to="/docs/intro">
                <svg
                  width="24"
                  height="24"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  className={styles.buttonIcon}
                >
                  <path d="M2 3h6a4 4 0 0 1 4 4v14a3 3 0 0 0-3-3H2z" />
                  <path d="M22 3h-6a4 4 0 0 0-4 4v14a3 3 0 0 1 3-3h7z" />
                </svg>
                <Translate id="homepage.hero.button">Start Learning</Translate>
                <svg
                  width="20"
                  height="20"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="2.5"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  className={styles.arrowIcon}
                >
                  <path d="M5 12h14" />
                  <path d="m12 5 7 7-7 7" />
                </svg>
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
          <Translate id="homepage.module.explore">Explore Module</Translate>
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
          <Translate id="homepage.journey.title">From Learning to Creation</Translate>
        </Heading>
        <div className={styles.journeyAnimation}>
          <img
            src={journeyGif}
            alt={translate({
              id: 'homepage.journey.imageAlt',
              message: 'Animation showing the complete journey: A human learns from the Physical AI book, gains knowledge of ROS2, Gazebo, Isaac, and VLA, then builds and assembles a humanoid robot which comes to life and works autonomously',
            })}
            className={styles.journeyGif}
          />
        </div>
        <p className="text--center margin-top--md">
          <Translate id="homepage.journey.description">
            Learn Physical AI concepts, build your own humanoid robot, and bring it to life!
          </Translate>
        </p>
      </div>
    </section>
  );
}

function ModuleCards(): ReactNode {
  const modules: ModuleInfo[] = [
    {
      title: translate({id: 'homepage.module1.title', message: 'Module 1: ROS2 Fundamentals'}),
      description: translate({id: 'homepage.module1.description', message: 'Master the Robot Operating System 2 - nodes, topics, services, actions, and Python integration with rclpy.'}),
      link: '/docs/module-1-ros2',
      icon: '/img/icons/ros2-icon.svg',
    },
    {
      title: translate({id: 'homepage.module2.title', message: 'Module 2: Robot Simulation'}),
      description: translate({id: 'homepage.module2.description', message: 'Learn Gazebo simulation, physics engines, sensor modeling, and URDF robot descriptions.'}),
      link: '/docs/module-2-simulation',
      icon: '/img/icons/simulation-icon.svg',
    },
    {
      title: translate({id: 'homepage.module3.title', message: 'Module 3: NVIDIA Isaac'}),
      description: translate({id: 'homepage.module3.description', message: 'GPU-accelerated robotics with Isaac Sim, Isaac ROS, photorealistic simulation, and edge deployment.'}),
      link: '/docs/module-3-nvidia-isaac',
      icon: '/img/icons/isaac-icon.svg',
    },
    {
      title: translate({id: 'homepage.module4.title', message: 'Module 4: VLA Models'}),
      description: translate({id: 'homepage.module4.description', message: 'Vision-Language-Action models, voice commands, LLM integration, and natural language robot control.'}),
      link: '/docs/module-4-vla',
      icon: '/img/icons/vla-icon.svg',
    },
  ];

  return (
    <section className={styles.modules}>
      <div className="container">
        <Heading as="h2" className="text--center margin-bottom--lg">
          <Translate id="homepage.modules.title">Course Modules</Translate>
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
