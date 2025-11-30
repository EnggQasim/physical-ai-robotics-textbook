/**
 * Personalization Components
 *
 * Provides user onboarding, content simplification, progress tracking,
 * and personalized recommendations.
 */
import React, { useState, useEffect, createContext, useContext } from 'react';
import { useAuth } from '../Auth';
import styles from './styles.module.css';

// Types
interface UserPreferences {
  user_id: string;
  role: string;
  experience_level: string;
  interests: string[];
  created_at: string;
  updated_at: string;
}

interface ReadingProgress {
  user_id: string;
  chapters: Record<string, {
    time_spent: number;
    completed: boolean;
    first_visit: string;
    last_visit: string | null;
  }>;
  total_time_seconds: number;
  last_activity: string | null;
}

interface Recommendation {
  chapter_id: string;
  score: number;
  reason: string;
}

interface OnboardingStatus {
  needs_onboarding: boolean;
  available_roles: Record<string, string>;
  available_levels: Record<string, string>;
}

interface PersonalizationContextType {
  preferences: UserPreferences | null;
  progress: ReadingProgress | null;
  recommendations: Recommendation[];
  isLoading: boolean;
  needsOnboarding: boolean;
  showOnboarding: boolean;
  setShowOnboarding: (show: boolean) => void;
  savePreferences: (role: string, level: string, interests?: string[]) => Promise<void>;
  updateProgress: (chapterId: string, timeSpent?: number, completed?: boolean) => Promise<void>;
  refreshRecommendations: (currentChapter?: string) => Promise<void>;
  getSimplifiedContent: (content: string, chapterId: string) => Promise<string>;
}

// API URL configuration
const API_URL = process.env.NODE_ENV === 'development'
  ? 'http://localhost:8000'
  : 'https://mqasim077-physical-ai-textbook-api.hf.space';

// Personalization Context
const PersonalizationContext = createContext<PersonalizationContextType | null>(null);

export function usePersonalization(): PersonalizationContextType {
  const context = useContext(PersonalizationContext);
  if (!context) {
    throw new Error('usePersonalization must be used within a PersonalizationProvider');
  }
  return context;
}

// Personalization Provider Component
export function PersonalizationProvider({ children }: { children: React.ReactNode }): JSX.Element {
  const { isAuthenticated, user } = useAuth();
  const [preferences, setPreferences] = useState<UserPreferences | null>(null);
  const [progress, setProgress] = useState<ReadingProgress | null>(null);
  const [recommendations, setRecommendations] = useState<Recommendation[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [needsOnboarding, setNeedsOnboarding] = useState(false);
  const [showOnboarding, setShowOnboarding] = useState(false);

  // Fetch personalization status on auth change
  useEffect(() => {
    const fetchStatus = async () => {
      if (!isAuthenticated) {
        setPreferences(null);
        setProgress(null);
        setNeedsOnboarding(false);
        setIsLoading(false);
        return;
      }

      try {
        const statusRes = await fetch(`${API_URL}/api/personalization/status`, {
          credentials: 'include'
        });
        const statusData: OnboardingStatus = await statusRes.json();
        setNeedsOnboarding(statusData.needs_onboarding);

        if (statusData.needs_onboarding) {
          setShowOnboarding(true);
        } else {
          // Fetch existing preferences
          const prefsRes = await fetch(`${API_URL}/api/personalization/preferences`, {
            credentials: 'include'
          });
          if (prefsRes.ok) {
            const prefsData = await prefsRes.json();
            if (prefsData) {
              setPreferences(prefsData);
            }
          }

          // Fetch progress
          const progressRes = await fetch(`${API_URL}/api/personalization/progress`, {
            credentials: 'include'
          });
          if (progressRes.ok) {
            setProgress(await progressRes.json());
          }
        }
      } catch (error) {
        console.error('Failed to fetch personalization status:', error);
      } finally {
        setIsLoading(false);
      }
    };

    fetchStatus();
  }, [isAuthenticated, user]);

  const savePreferences = async (role: string, level: string, interests: string[] = []) => {
    try {
      const res = await fetch(`${API_URL}/api/personalization/preferences`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({ role, experience_level: level, interests })
      });

      if (res.ok) {
        const data = await res.json();
        setPreferences(data);
        setNeedsOnboarding(false);
        setShowOnboarding(false);
      }
    } catch (error) {
      console.error('Failed to save preferences:', error);
    }
  };

  const updateProgress = async (chapterId: string, timeSpent = 0, completed = false) => {
    if (!isAuthenticated) return;

    try {
      const res = await fetch(`${API_URL}/api/personalization/progress`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({
          chapter_id: chapterId,
          time_spent_seconds: timeSpent,
          completed
        })
      });

      if (res.ok) {
        setProgress(await res.json());
      }
    } catch (error) {
      console.error('Failed to update progress:', error);
    }
  };

  const refreshRecommendations = async (currentChapter?: string) => {
    try {
      const url = new URL(`${API_URL}/api/personalization/recommendations`);
      if (currentChapter) {
        url.searchParams.set('current_chapter', currentChapter);
      }

      const res = await fetch(url.toString(), { credentials: 'include' });
      if (res.ok) {
        setRecommendations(await res.json());
      }
    } catch (error) {
      console.error('Failed to fetch recommendations:', error);
    }
  };

  const getSimplifiedContent = async (content: string, chapterId: string): Promise<string> => {
    try {
      const res = await fetch(`${API_URL}/api/personalization/simplify`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({ content, chapter_id: chapterId })
      });

      if (res.ok) {
        const data = await res.json();
        return data.simplified_content;
      }
    } catch (error) {
      console.error('Failed to simplify content:', error);
    }
    return content;
  };

  return (
    <PersonalizationContext.Provider
      value={{
        preferences,
        progress,
        recommendations,
        isLoading,
        needsOnboarding,
        showOnboarding,
        setShowOnboarding,
        savePreferences,
        updateProgress,
        refreshRecommendations,
        getSimplifiedContent
      }}
    >
      {children}
    </PersonalizationContext.Provider>
  );
}

// Onboarding Modal Component
export function OnboardingModal(): JSX.Element | null {
  const { showOnboarding, setShowOnboarding, savePreferences, isLoading } = usePersonalization();
  const { isAuthenticated } = useAuth();
  const [step, setStep] = useState(1);
  const [role, setRole] = useState('');
  const [level, setLevel] = useState('');
  const [interests, setInterests] = useState<string[]>([]);
  const [isSaving, setIsSaving] = useState(false);

  const roles = [
    { id: 'student', label: 'Student', desc: 'Learning robotics for academic or personal projects' },
    { id: 'researcher', label: 'Researcher', desc: 'Working on cutting-edge AI and robotics research' },
    { id: 'engineer', label: 'Engineer', desc: 'Building production robotics systems' },
    { id: 'hobbyist', label: 'Hobbyist', desc: 'Building robots as a hobby' }
  ];

  const levels = [
    { id: 'beginner', label: 'Beginner', desc: 'New to robotics and AI' },
    { id: 'intermediate', label: 'Intermediate', desc: 'Some programming experience' },
    { id: 'advanced', label: 'Advanced', desc: 'Experienced developer' }
  ];

  const interestOptions = [
    'ROS2', 'Simulation', 'NVIDIA Isaac', 'Voice Control', 'LLM Integration',
    'Computer Vision', 'Motion Planning', 'Reinforcement Learning'
  ];

  if (!isAuthenticated || !showOnboarding || isLoading) {
    return null;
  }

  const handleComplete = async () => {
    if (!role || !level) return;
    setIsSaving(true);
    await savePreferences(role, level, interests);
    setIsSaving(false);
  };

  const toggleInterest = (interest: string) => {
    setInterests(prev =>
      prev.includes(interest)
        ? prev.filter(i => i !== interest)
        : [...prev, interest]
    );
  };

  return (
    <div className={styles.modalOverlay}>
      <div className={styles.modal}>
        <div className={styles.modalHeader}>
          <h2>Welcome to Physical AI Textbook!</h2>
          <p>Let us personalize your learning experience</p>
        </div>

        <div className={styles.stepIndicator}>
          <div className={`${styles.step} ${step >= 1 ? styles.active : ''}`}>1</div>
          <div className={styles.stepLine} />
          <div className={`${styles.step} ${step >= 2 ? styles.active : ''}`}>2</div>
          <div className={styles.stepLine} />
          <div className={`${styles.step} ${step >= 3 ? styles.active : ''}`}>3</div>
        </div>

        {step === 1 && (
          <div className={styles.stepContent}>
            <h3>What describes you best?</h3>
            <div className={styles.optionGrid}>
              {roles.map(r => (
                <button
                  key={r.id}
                  className={`${styles.optionCard} ${role === r.id ? styles.selected : ''}`}
                  onClick={() => setRole(r.id)}
                >
                  <strong>{r.label}</strong>
                  <span>{r.desc}</span>
                </button>
              ))}
            </div>
            <div className={styles.actions}>
              <button
                className={styles.skipButton}
                onClick={() => setShowOnboarding(false)}
              >
                Skip for now
              </button>
              <button
                className={styles.nextButton}
                disabled={!role}
                onClick={() => setStep(2)}
              >
                Next
              </button>
            </div>
          </div>
        )}

        {step === 2 && (
          <div className={styles.stepContent}>
            <h3>What's your experience level?</h3>
            <div className={styles.optionGrid}>
              {levels.map(l => (
                <button
                  key={l.id}
                  className={`${styles.optionCard} ${level === l.id ? styles.selected : ''}`}
                  onClick={() => setLevel(l.id)}
                >
                  <strong>{l.label}</strong>
                  <span>{l.desc}</span>
                </button>
              ))}
            </div>
            <div className={styles.actions}>
              <button className={styles.backButton} onClick={() => setStep(1)}>
                Back
              </button>
              <button
                className={styles.nextButton}
                disabled={!level}
                onClick={() => setStep(3)}
              >
                Next
              </button>
            </div>
          </div>
        )}

        {step === 3 && (
          <div className={styles.stepContent}>
            <h3>What topics interest you? (optional)</h3>
            <div className={styles.interestGrid}>
              {interestOptions.map(interest => (
                <button
                  key={interest}
                  className={`${styles.interestChip} ${interests.includes(interest) ? styles.selected : ''}`}
                  onClick={() => toggleInterest(interest)}
                >
                  {interest}
                </button>
              ))}
            </div>
            <div className={styles.actions}>
              <button className={styles.backButton} onClick={() => setStep(2)}>
                Back
              </button>
              <button
                className={styles.completeButton}
                onClick={handleComplete}
                disabled={isSaving}
              >
                {isSaving ? 'Saving...' : 'Complete Setup'}
              </button>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

// Simplified Toggle Component
export function SimplifiedToggle({
  content,
  chapterId,
  className = ''
}: {
  content: string;
  chapterId: string;
  className?: string;
}): JSX.Element {
  const { preferences, getSimplifiedContent, isLoading } = usePersonalization();
  const { isAuthenticated } = useAuth();
  const [showSimplified, setShowSimplified] = useState(false);
  const [simplifiedContent, setSimplifiedContent] = useState<string | null>(null);
  const [isLoadingSimplified, setIsLoadingSimplified] = useState(false);

  // Only show for beginner users
  const showToggle = isAuthenticated && preferences?.experience_level === 'beginner';

  const handleToggle = async () => {
    if (!showSimplified && !simplifiedContent) {
      setIsLoadingSimplified(true);
      const simplified = await getSimplifiedContent(content, chapterId);
      setSimplifiedContent(simplified);
      setIsLoadingSimplified(false);
    }
    setShowSimplified(!showSimplified);
  };

  if (!showToggle || isLoading) {
    return <div className={className}>{content}</div>;
  }

  return (
    <div className={`${styles.simplifiedContainer} ${className}`}>
      <button className={styles.simplifyToggle} onClick={handleToggle}>
        {isLoadingSimplified ? (
          <span>Loading...</span>
        ) : showSimplified ? (
          <>
            <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M4 12h16M12 4v16" />
            </svg>
            Show Technical
          </>
        ) : (
          <>
            <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <circle cx="12" cy="12" r="10" />
              <path d="M12 16v-4M12 8h.01" />
            </svg>
            Simplify
          </>
        )}
      </button>
      <div>{showSimplified && simplifiedContent ? simplifiedContent : content}</div>
    </div>
  );
}

// Progress Indicator Component
export function ProgressIndicator({
  chapterId,
  className = ''
}: {
  chapterId: string;
  className?: string;
}): JSX.Element | null {
  const { progress, isLoading } = usePersonalization();
  const { isAuthenticated } = useAuth();

  if (!isAuthenticated || isLoading || !progress) {
    return null;
  }

  const chapterProgress = progress.chapters[chapterId];
  if (!chapterProgress) {
    return null;
  }

  const { completed, time_spent } = chapterProgress;
  const minutes = Math.round(time_spent / 60);

  return (
    <div className={`${styles.progressIndicator} ${className}`}>
      {completed ? (
        <span className={styles.completed}>
          <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <polyline points="20 6 9 17 4 12" />
          </svg>
          Completed
        </span>
      ) : minutes > 0 ? (
        <span className={styles.inProgress}>
          {minutes} min read
        </span>
      ) : null}
    </div>
  );
}

// Recommended Next Component
export function RecommendedNext({
  currentChapter,
  className = ''
}: {
  currentChapter?: string;
  className?: string;
}): JSX.Element | null {
  const { recommendations, refreshRecommendations, isLoading } = usePersonalization();
  const { isAuthenticated } = useAuth();

  useEffect(() => {
    if (isAuthenticated) {
      refreshRecommendations(currentChapter);
    }
  }, [isAuthenticated, currentChapter]);

  if (isLoading || recommendations.length === 0) {
    return null;
  }

  // Chapter ID to URL mapping
  const chapterUrls: Record<string, string> = {
    'intro': '/docs/intro',
    'module-1-ros2-index': '/docs/module-1-ros2-fundamentals/',
    'module-1-ros2-nodes-topics': '/docs/module-1-ros2-fundamentals/nodes-topics',
    'module-1-ros2-services-actions': '/docs/module-1-ros2-fundamentals/services-actions',
    'module-1-ros2-rclpy': '/docs/module-1-ros2-fundamentals/rclpy',
    'module-2-simulation-index': '/docs/module-2-robot-simulation/',
    'module-2-simulation-gazebo': '/docs/module-2-robot-simulation/gazebo',
    'module-2-simulation-urdf': '/docs/module-2-robot-simulation/urdf',
    'module-3-nvidia-isaac-index': '/docs/module-3-nvidia-isaac/',
    'module-3-nvidia-isaac-sim': '/docs/module-3-nvidia-isaac/isaac-sim',
    'module-3-nvidia-isaac-ros': '/docs/module-3-nvidia-isaac/isaac-ros',
    'module-4-vla-index': '/docs/module-4-vla-models/',
    'module-4-vla-voice-commands': '/docs/module-4-vla-models/voice-commands',
    'module-4-vla-llm-integration': '/docs/module-4-vla-models/llm-integration'
  };

  const formatChapterName = (id: string): string => {
    return id
      .replace('module-', 'Module ')
      .replace(/-/g, ' ')
      .replace(/\b\w/g, c => c.toUpperCase());
  };

  return (
    <div className={`${styles.recommendedNext} ${className}`}>
      <h4>Recommended Next</h4>
      <div className={styles.recommendationList}>
        {recommendations.slice(0, 3).map(rec => (
          <a
            key={rec.chapter_id}
            href={chapterUrls[rec.chapter_id] || '#'}
            className={styles.recommendationCard}
          >
            <strong>{formatChapterName(rec.chapter_id)}</strong>
            <span>{rec.reason}</span>
          </a>
        ))}
      </div>
    </div>
  );
}

export default {
  PersonalizationProvider,
  OnboardingModal,
  SimplifiedToggle,
  ProgressIndicator,
  RecommendedNext,
  usePersonalization
};
