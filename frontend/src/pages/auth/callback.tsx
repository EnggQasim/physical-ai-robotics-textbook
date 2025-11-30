import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';

const API_URL = 'https://mqasim077-physical-ai-textbook-api.hf.space';

export default function AuthCallback(): JSX.Element {
  const [status, setStatus] = useState<'loading' | 'success' | 'error'>('loading');
  const [errorMessage, setErrorMessage] = useState('');

  useEffect(() => {
    const handleCallback = async () => {
      const params = new URLSearchParams(window.location.search);
      const code = params.get('code');
      const state = params.get('state');
      const error = params.get('error');

      if (error) {
        setStatus('error');
        setErrorMessage('Authentication was cancelled or failed: ' + error);
        setTimeout(() => {
          window.location.href = '/physical-ai-robotics-textbook/';
        }, 3000);
        return;
      }

      if (!code) {
        setStatus('error');
        setErrorMessage('No authorization code received');
        setTimeout(() => {
          window.location.href = '/physical-ai-robotics-textbook/';
        }, 3000);
        return;
      }

      try {
        // Exchange code for token via backend
        const response = await fetch(
          `${API_URL}/api/auth/github/callback?code=${code}&state=${state || ''}`,
          {
            method: 'GET',
            credentials: 'include',
          }
        );

        if (!response.ok) {
          const data = await response.json();
          throw new Error(data.detail || 'Authentication failed');
        }

        const data = await response.json();

        // Store token in localStorage for the frontend
        if (data.access_token) {
          localStorage.setItem('auth_token', data.access_token);
        }
        if (data.user) {
          localStorage.setItem('auth_user', JSON.stringify(data.user));
        }

        setStatus('success');

        // Redirect to home
        setTimeout(() => {
          window.location.href = '/physical-ai-robotics-textbook/';
        }, 1500);
      } catch (err) {
        setStatus('error');
        setErrorMessage(err instanceof Error ? err.message : 'Authentication failed');
        setTimeout(() => {
          window.location.href = '/physical-ai-robotics-textbook/';
        }, 3000);
      }
    };

    handleCallback();
  }, []);

  return (
    <Layout title="Authenticating..." description="Processing authentication">
      <div
        style={{
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          minHeight: '50vh',
          flexDirection: 'column',
          gap: '1rem',
          padding: '2rem',
        }}
      >
        {status === 'loading' && (
          <>
            <div
              style={{
                width: '50px',
                height: '50px',
                border: '4px solid rgba(118, 185, 0, 0.2)',
                borderTopColor: '#76b900',
                borderRadius: '50%',
                animation: 'spin 1s linear infinite',
              }}
            />
            <h2>Authenticating...</h2>
            <p style={{ color: 'var(--ifm-color-emphasis-600)' }}>
              Please wait while we complete your sign-in.
            </p>
          </>
        )}

        {status === 'success' && (
          <>
            <svg
              width="60"
              height="60"
              viewBox="0 0 24 24"
              fill="none"
              stroke="#76b900"
              strokeWidth="2"
            >
              <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14" />
              <polyline points="22 4 12 14.01 9 11.01" />
            </svg>
            <h2>Authentication Successful!</h2>
            <p style={{ color: 'var(--ifm-color-emphasis-600)' }}>
              Redirecting you back to the textbook...
            </p>
          </>
        )}

        {status === 'error' && (
          <>
            <svg
              width="60"
              height="60"
              viewBox="0 0 24 24"
              fill="none"
              stroke="#ff6b6b"
              strokeWidth="2"
            >
              <circle cx="12" cy="12" r="10" />
              <line x1="15" y1="9" x2="9" y2="15" />
              <line x1="9" y1="9" x2="15" y2="15" />
            </svg>
            <h2>Authentication Failed</h2>
            <p style={{ color: '#ff6b6b' }}>{errorMessage}</p>
            <p style={{ color: 'var(--ifm-color-emphasis-600)' }}>
              Redirecting you back...
            </p>
          </>
        )}

        <style>{`
          @keyframes spin {
            to { transform: rotate(360deg); }
          }
        `}</style>
      </div>
    </Layout>
  );
}
