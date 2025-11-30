import React, { useState, useEffect, createContext, useContext, useCallback } from 'react';
import styles from './styles.module.css';

// Types
interface User {
  user_id: string;
  email?: string;
  name?: string;
  avatar_url?: string;
  provider?: string;
  created_at?: string;
  last_login?: string;
}

interface AuthContextType {
  user: User | null;
  isLoading: boolean;
  isAuthenticated: boolean;
  signInWithGitHub: () => Promise<void>;
  signInWithGoogle: () => Promise<void>;
  signOut: () => Promise<void>;
  refreshSession: () => Promise<void>;
}

interface AuthStatus {
  github_configured: boolean;
  google_configured: boolean;
  any_configured: boolean;
}

// API URL configuration
const API_URL = process.env.NODE_ENV === 'development'
  ? 'http://localhost:8000'
  : 'https://mqasim077-physical-ai-textbook-api.hf.space';

// Auth Context
const AuthContext = createContext<AuthContextType | null>(null);

export function useAuth(): AuthContextType {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}

// Auth Provider Component
export function AuthProvider({ children }: { children: React.ReactNode }): JSX.Element {
  const [user, setUser] = useState<User | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  const refreshSession = useCallback(async () => {
    try {
      const response = await fetch(`${API_URL}/api/auth/session`, {
        credentials: 'include'
      });
      const data = await response.json();

      if (data.authenticated && data.user) {
        setUser(data.user);
      } else {
        setUser(null);
      }
    } catch (error) {
      console.error('Failed to refresh session:', error);
      setUser(null);
    } finally {
      setIsLoading(false);
    }
  }, []);

  useEffect(() => {
    refreshSession();
  }, [refreshSession]);

  const signInWithGitHub = async () => {
    const redirectUri = `${window.location.origin}/auth/callback`;
    const response = await fetch(
      `${API_URL}/api/auth/github/url?redirect_uri=${encodeURIComponent(redirectUri)}`
    );
    const data = await response.json();

    if (data.url) {
      window.location.href = data.url;
    }
  };

  const signInWithGoogle = async () => {
    const redirectUri = `${window.location.origin}/auth/callback`;
    const response = await fetch(
      `${API_URL}/api/auth/google/url?redirect_uri=${encodeURIComponent(redirectUri)}`
    );
    const data = await response.json();

    if (data.url) {
      window.location.href = data.url;
    }
  };

  const signOut = async () => {
    try {
      await fetch(`${API_URL}/api/auth/signout`, {
        method: 'POST',
        credentials: 'include'
      });
      setUser(null);
    } catch (error) {
      console.error('Sign out failed:', error);
    }
  };

  return (
    <AuthContext.Provider
      value={{
        user,
        isLoading,
        isAuthenticated: !!user,
        signInWithGitHub,
        signInWithGoogle,
        signOut,
        refreshSession
      }}
    >
      {children}
    </AuthContext.Provider>
  );
}

// Sign In Button Component
export function SignInButton(): JSX.Element {
  const [authStatus, setAuthStatus] = useState<AuthStatus | null>(null);
  const [showDropdown, setShowDropdown] = useState(false);
  const { signInWithGitHub, signInWithGoogle, isLoading } = useAuth();

  useEffect(() => {
    const fetchAuthStatus = async () => {
      try {
        const response = await fetch(`${API_URL}/api/auth/status`);
        const data = await response.json();
        setAuthStatus(data);
      } catch (error) {
        console.error('Failed to fetch auth status:', error);
      }
    };
    fetchAuthStatus();
  }, []);

  if (isLoading) {
    return <div className={styles.loadingPlaceholder} />;
  }

  if (!authStatus?.any_configured) {
    return null; // Don't show sign in if no providers configured
  }

  return (
    <div className={styles.signInContainer}>
      <button
        className={styles.signInButton}
        onClick={() => setShowDropdown(!showDropdown)}
        aria-label="Sign in options"
      >
        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2" />
          <circle cx="12" cy="7" r="4" />
        </svg>
        <span>Sign In</span>
      </button>

      {showDropdown && (
        <div className={styles.dropdown}>
          {authStatus?.github_configured && (
            <button
              className={styles.providerButton}
              onClick={() => {
                setShowDropdown(false);
                signInWithGitHub();
              }}
            >
              <svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor">
                <path d="M12 0C5.37 0 0 5.37 0 12c0 5.31 3.435 9.795 8.205 11.385.6.105.825-.255.825-.57 0-.285-.015-1.23-.015-2.235-3.015.555-3.795-.735-4.035-1.41-.135-.345-.72-1.41-1.23-1.695-.42-.225-1.02-.78-.015-.795.945-.015 1.62.87 1.845 1.23 1.08 1.815 2.805 1.305 3.495.99.105-.78.42-1.305.765-1.605-2.67-.3-5.46-1.335-5.46-5.925 0-1.305.465-2.385 1.23-3.225-.12-.3-.54-1.53.12-3.18 0 0 1.005-.315 3.3 1.23.96-.27 1.98-.405 3-.405s2.04.135 3 .405c2.295-1.56 3.3-1.23 3.3-1.23.66 1.65.24 2.88.12 3.18.765.84 1.23 1.905 1.23 3.225 0 4.605-2.805 5.625-5.475 5.925.435.375.81 1.095.81 2.22 0 1.605-.015 2.895-.015 3.3 0 .315.225.69.825.57A12.02 12.02 0 0024 12c0-6.63-5.37-12-12-12z" />
              </svg>
              <span>Continue with GitHub</span>
            </button>
          )}

          {authStatus?.google_configured && (
            <button
              className={styles.providerButton}
              onClick={() => {
                setShowDropdown(false);
                signInWithGoogle();
              }}
            >
              <svg width="18" height="18" viewBox="0 0 24 24">
                <path fill="#4285F4" d="M22.56 12.25c0-.78-.07-1.53-.2-2.25H12v4.26h5.92c-.26 1.37-1.04 2.53-2.21 3.31v2.77h3.57c2.08-1.92 3.28-4.74 3.28-8.09z" />
                <path fill="#34A853" d="M12 23c2.97 0 5.46-.98 7.28-2.66l-3.57-2.77c-.98.66-2.23 1.06-3.71 1.06-2.86 0-5.29-1.93-6.16-4.53H2.18v2.84C3.99 20.53 7.7 23 12 23z" />
                <path fill="#FBBC05" d="M5.84 14.09c-.22-.66-.35-1.36-.35-2.09s.13-1.43.35-2.09V7.07H2.18C1.43 8.55 1 10.22 1 12s.43 3.45 1.18 4.93l2.85-2.22.81-.62z" />
                <path fill="#EA4335" d="M12 5.38c1.62 0 3.06.56 4.21 1.64l3.15-3.15C17.45 2.09 14.97 1 12 1 7.7 1 3.99 3.47 2.18 7.07l3.66 2.84c.87-2.6 3.3-4.53 6.16-4.53z" />
              </svg>
              <span>Continue with Google</span>
            </button>
          )}
        </div>
      )}
    </div>
  );
}

// User Menu Component
export function UserMenu(): JSX.Element {
  const { user, signOut, isLoading } = useAuth();
  const [showDropdown, setShowDropdown] = useState(false);

  if (isLoading) {
    return <div className={styles.loadingPlaceholder} />;
  }

  if (!user) {
    return <SignInButton />;
  }

  return (
    <div className={styles.userMenuContainer}>
      <button
        className={styles.avatarButton}
        onClick={() => setShowDropdown(!showDropdown)}
        aria-label="User menu"
      >
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
      </button>

      {showDropdown && (
        <div className={styles.dropdown}>
          <div className={styles.userInfo}>
            <strong>{user.name || 'User'}</strong>
            {user.email && <span className={styles.email}>{user.email}</span>}
          </div>

          <div className={styles.divider} />

          <a href="/profile" className={styles.menuItem}>
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2" />
              <circle cx="12" cy="7" r="4" />
            </svg>
            <span>Profile</span>
          </a>

          <button
            className={styles.menuItem}
            onClick={() => {
              setShowDropdown(false);
              signOut();
            }}
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M9 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h4" />
              <polyline points="16 17 21 12 16 7" />
              <line x1="21" y1="12" x2="9" y2="12" />
            </svg>
            <span>Sign Out</span>
          </button>
        </div>
      )}
    </div>
  );
}

// Export default component
export default function Auth(): JSX.Element {
  const { isAuthenticated } = useAuth();

  return isAuthenticated ? <UserMenu /> : <SignInButton />;
}
