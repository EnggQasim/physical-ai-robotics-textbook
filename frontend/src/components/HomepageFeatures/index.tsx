import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  icon: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Industry-Standard Tools',
    icon: '🔧',
    description: (
      <>
        Learn ROS2, the foundation of modern robotics used by companies worldwide.
        Build real robot applications with battle-tested frameworks.
      </>
    ),
  },
  {
    title: 'GPU-Accelerated Simulation',
    icon: '🎯',
    description: (
      <>
        Master NVIDIA Isaac Sim for photorealistic simulation, synthetic data
        generation, and seamless sim-to-real transfer.
      </>
    ),
  },
  {
    title: 'Cutting-Edge AI',
    icon: '🧠',
    description: (
      <>
        Implement Vision-Language-Action models to control robots with natural
        language commands and multimodal understanding.
      </>
    ),
  },
];

function Feature({title, icon, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <div className={styles.featureIcon}>{icon}</div>
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
