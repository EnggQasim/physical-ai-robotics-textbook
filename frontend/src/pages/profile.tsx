import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '@site/src/components/Auth';
import { usePersonalization } from '@site/src/components/Personalization';
import styles from './profile.module.css';

export default function ProfilePage(): JSX.Element {
  const { user, isLoading, isAuthenticated, signOut } = useAuth();
  const {
    preferences,
    progress,
    setShowOnboarding,
    isLoading: isLoadingPrefs
  } = usePersonalization();
  const [showPreferences, setShowPreferences] = useState(false);

  if (isLoading) {
    return (
      <Layout title="Profile" description="Your profile page">
        <div className={styles.container}>
          <div className={styles.loading}>
            <div className={styles.spinner} />
            <p>Loading...</p>
          </div>
        </div>
      </Layout>
    );
  }

  if (!isAuthenticated) {
    return (
      <Layout title="Profile" description="Your profile page">
        <div className={styles.container}>
          <div className={styles.notSignedIn}>
            <svg width="64" height="64" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
              <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2" />
              <circle cx="12" cy="7" r="4" />
            </svg>
            <h1>Sign In Required</h1>
            <p>Please sign in to view your profile.</p>
            <p className={styles.hint}>
              Click the Sign In button in the navbar to get started.
            </p>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Profile" description="Your profile page">
      <div className={styles.container}>
        <div className={styles.profileCard}>
          <div className={styles.header}>
            {user.avatar_url ? (
              <img
                src={user.avatar_url}
                alt={user.name || 'User avatar'}
                className={styles.avatar}
              />
            ) : (
              <div className={styles.avatarPlaceholder}>
                {(user.name || user.email || 'U')[0].toUpperCase()}
              </div>
            )}
            <div className={styles.headerInfo}>
              <h1>{user.name || 'User'}</h1>
              {user.email && <p className={styles.email}>{user.email}</p>}
            </div>
          </div>

          <div className={styles.details}>
            <div className={styles.detailItem}>
              <span className={styles.label}>Provider</span>
              <span className={styles.value}>
                {user.provider === 'github' ? (
                  <span className={styles.providerBadge}>
                    <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor">
                      <path d="M12 0C5.37 0 0 5.37 0 12c0 5.31 3.435 9.795 8.205 11.385.6.105.825-.255.825-.57 0-.285-.015-1.23-.015-2.235-3.015.555-3.795-.735-4.035-1.41-.135-.345-.72-1.41-1.23-1.695-.42-.225-1.02-.78-.015-.795.945-.015 1.62.87 1.845 1.23 1.08 1.815 2.805 1.305 3.495.99.105-.78.42-1.305.765-1.605-2.67-.3-5.46-1.335-5.46-5.925 0-1.305.465-2.385 1.23-3.225-.12-.3-.54-1.53.12-3.18 0 0 1.005-.315 3.3 1.23.96-.27 1.98-.405 3-.405s2.04.135 3 .405c2.295-1.56 3.3-1.23 3.3-1.23.66 1.65.24 2.88.12 3.18.765.84 1.23 1.905 1.23 3.225 0 4.605-2.805 5.625-5.475 5.925.435.375.81 1.095.81 2.22 0 1.605-.015 2.895-.015 3.3 0 .315.225.69.825.57A12.02 12.02 0 0024 12c0-6.63-5.37-12-12-12z" />
                    </svg>
                    GitHub
                  </span>
                ) : user.provider === 'google' ? (
                  <span className={styles.providerBadge}>
                    <svg width="16" height="16" viewBox="0 0 24 24">
                      <path fill="#4285F4" d="M22.56 12.25c0-.78-.07-1.53-.2-2.25H12v4.26h5.92c-.26 1.37-1.04 2.53-2.21 3.31v2.77h3.57c2.08-1.92 3.28-4.74 3.28-8.09z" />
                      <path fill="#34A853" d="M12 23c2.97 0 5.46-.98 7.28-2.66l-3.57-2.77c-.98.66-2.23 1.06-3.71 1.06-2.86 0-5.29-1.93-6.16-4.53H2.18v2.84C3.99 20.53 7.7 23 12 23z" />
                      <path fill="#FBBC05" d="M5.84 14.09c-.22-.66-.35-1.36-.35-2.09s.13-1.43.35-2.09V7.07H2.18C1.43 8.55 1 10.22 1 12s.43 3.45 1.18 4.93l2.85-2.22.81-.62z" />
                      <path fill="#EA4335" d="M12 5.38c1.62 0 3.06.56 4.21 1.64l3.15-3.15C17.45 2.09 14.97 1 12 1 7.7 1 3.99 3.47 2.18 7.07l3.66 2.84c.87-2.6 3.3-4.53 6.16-4.53z" />
                    </svg>
                    Google
                  </span>
                ) : (
                  user.provider || 'Unknown'
                )}
              </span>
            </div>

            <div className={styles.detailItem}>
              <span className={styles.label}>User ID</span>
              <span className={styles.value}>{user.user_id}</span>
            </div>

            {user.created_at && (
              <div className={styles.detailItem}>
                <span className={styles.label}>Member Since</span>
                <span className={styles.value}>
                  {new Date(user.created_at).toLocaleDateString('en-US', {
                    year: 'numeric',
                    month: 'long',
                    day: 'numeric'
                  })}
                </span>
              </div>
            )}

            {user.last_login && (
              <div className={styles.detailItem}>
                <span className={styles.label}>Last Login</span>
                <span className={styles.value}>
                  {new Date(user.last_login).toLocaleDateString('en-US', {
                    year: 'numeric',
                    month: 'long',
                    day: 'numeric',
                    hour: '2-digit',
                    minute: '2-digit'
                  })}
                </span>
              </div>
            )}
          </div>

          {/* Learning Preferences Section */}
          <div className={styles.preferencesSection}>
            <div className={styles.sectionHeader}>
              <h2>Learning Preferences</h2>
              {preferences && (
                <button
                  className={styles.editButton}
                  onClick={() => setShowOnboarding(true)}
                >
                  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <path d="M11 4H4a2 2 0 0 0-2 2v14a2 2 0 0 0 2 2h14a2 2 0 0 0 2-2v-7" />
                    <path d="M18.5 2.5a2.121 2.121 0 0 1 3 3L12 15l-4 1 1-4 9.5-9.5z" />
                  </svg>
                  Edit
                </button>
              )}
            </div>

            {isLoadingPrefs ? (
              <div className={styles.loadingText}>Loading preferences...</div>
            ) : preferences ? (
              <div className={styles.preferencesGrid}>
                <div className={styles.prefItem}>
                  <span className={styles.label}>Role</span>
                  <span className={styles.prefBadge}>{preferences.role}</span>
                </div>
                <div className={styles.prefItem}>
                  <span className={styles.label}>Experience Level</span>
                  <span className={styles.prefBadge}>{preferences.experience_level}</span>
                </div>
                {preferences.interests && preferences.interests.length > 0 && (
                  <div className={styles.prefItem}>
                    <span className={styles.label}>Interests</span>
                    <div className={styles.interestTags}>
                      {preferences.interests.map(interest => (
                        <span key={interest} className={styles.interestTag}>{interest}</span>
                      ))}
                    </div>
                  </div>
                )}
              </div>
            ) : (
              <div className={styles.noPreferences}>
                <p>You haven't set up your learning preferences yet.</p>
                <button
                  className={styles.setupButton}
                  onClick={() => setShowOnboarding(true)}
                >
                  Set Up Preferences
                </button>
              </div>
            )}
          </div>

          {/* Reading Progress Section */}
          {progress && Object.keys(progress.chapters).length > 0 && (
            <div className={styles.progressSection}>
              <h2>Reading Progress</h2>
              <div className={styles.progressStats}>
                <div className={styles.statItem}>
                  <span className={styles.statValue}>
                    {Object.values(progress.chapters).filter(c => c.completed).length}
                  </span>
                  <span className={styles.statLabel}>Chapters Completed</span>
                </div>
                <div className={styles.statItem}>
                  <span className={styles.statValue}>
                    {Math.round(progress.total_time_seconds / 60)}
                  </span>
                  <span className={styles.statLabel}>Minutes Read</span>
                </div>
              </div>
            </div>
          )}

          <div className={styles.actions}>
            <button
              className={styles.signOutButton}
              onClick={signOut}
            >
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M9 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h4" />
                <polyline points="16 17 21 12 16 7" />
                <line x1="21" y1="12" x2="9" y2="12" />
              </svg>
              Sign Out
            </button>
          </div>
        </div>
      </div>
    </Layout>
  );
}
