import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title" style={{fontSize: '3rem', fontWeight: 'bold'}}>{siteConfig.title}</h1>
        <p className="hero__subtitle" style={{fontSize: '1.5rem'}}>{siteConfig.tagline}</p>
        <div style={{marginTop: '20px', fontSize: '1rem'}}>
          <p><strong>Editors:</strong> Dr. Alan Turing, Dr. Grace Hopper</p>
          <p><strong>Contributing Authors:</strong> The Docusaurus Community</p>
        </div>
        <div className={styles.buttons} style={{marginTop: '40px'}}>
          <Link
            className="button button--secondary button--lg"
            to="/modules/module-1-ros2/introduction-to-ros2">
            Start Reading
          </Link>
        </div>
      </div>
    </header>
  );
}

const tableOfContents = [
    { title: 'Module 1: ROS 2 & Controllers (rclpy)', link: '/modules/module-1-ros2/introduction-to-ros2' },
    { title: 'Module 2: Digital Twin (Gazebo & Unity)', link: '/modules/module-2-digital-twin/introduction-to-digital-twins' },
    { title: 'Module 3: AI-Robot Brain (NVIDIA Isaac)', link: '/modules/module-3-isaac/introduction-to-nvidia-isaac-ecosystem' },
    { title: 'Module 4: Vision-Language-Action (VLA)', link: '/modules/module-4-vla/understanding-vla-robotics' },
]

function TableOfContents() {
    return (
        <div className="container" style={{padding: '4rem 2rem', textAlign: 'left'}}>
            <h2 style={{fontSize: '2rem', marginBottom: '2rem', textAlign: 'center', borderBottom: '2px solid var(--ifm-color-emphasis-300)', paddingBottom: '10px'}}>Textbook Modules</h2>
            <div className="row">
                <div className="col col--10 col--offset-1">
                    <div className="module-grid">
                        {tableOfContents.map((item, index) => (
                            <div key={index} className="module-card">
                                <Link to={item.link} style={{textDecoration: 'none', color: 'inherit', display: 'block', height: '100%'}}>
                                    <h3 style={{margin: 0, color: 'var(--ifm-heading-color)', fontSize: '1.3rem'}}>{item.title}</h3>
                                </Link>
                            </div>
                        ))}
                    </div>
                </div>
            </div>
        </div>
    )
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="A Comprehensive Textbook on Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <TableOfContents />
      </main>
    </Layout>
  );
}