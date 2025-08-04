import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Hands-on Robotics',
    imgSrc: '/img/BlueRov_3_1.jpg',
    description: (
      <>
        Build and program real robots to solve environmental challenges.
      </>
    ),
  },
  {
    title: 'BlueBoat Environmental Platform',
    imgSrc: '/img/Pool_BlueBoat.jpg',
    description: (
      <>
        The BlueBoat is an autonomous surface vehicle used for real-world water quality monitoring.
      </>
    ),
  },
  {
    title: 'Mobile Sensing',
    imgSrc: '/img/BlueBoat_SVan.jpg',
    description: (
      <>
        Explore terrestrial environments with mobile sensor robots.
      </>
    ),
  },
];

function Feature({ title, imgSrc, description }) {
  return (
    <div className={clsx('col col--4', styles.feature)}>
      <div className="text--center">
        <img
          src={useBaseUrl(imgSrc)}
          alt={title}
          className={styles.featureImg}
        />
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
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
