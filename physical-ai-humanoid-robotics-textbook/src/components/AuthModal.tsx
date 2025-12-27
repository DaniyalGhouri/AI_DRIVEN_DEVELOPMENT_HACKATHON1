import React, { useState } from 'react';
import { loginUser, registerUser } from '../lib/ragClient';
import './AuthModal.css';

interface AuthModalProps {
  isOpen: boolean;
  onClose: () => void;
  onLoginSuccess: () => void;
}

const AuthModal: React.FC<AuthModalProps> = ({ isOpen, onClose, onLoginSuccess }) => {
  const [isLogin, setIsLogin] = useState(true);
  const [username, setUsername] = useState('');
  const [password, setPassword] = useState('');
  const [email, setEmail] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState('');
  const [hardwareBackground, setHardwareBackground] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  if (!isOpen) return null;

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setLoading(true);

    try {
      if (isLogin) {
        const tokenData = await loginUser(username, password);
        localStorage.setItem('access_token', tokenData.access_token);
        onLoginSuccess();
        onClose();
      } else {
        await registerUser(username, password, email, softwareBackground, hardwareBackground);
        // Auto login after register
        const tokenData = await loginUser(username, password);
        localStorage.setItem('access_token', tokenData.access_token);
        onLoginSuccess();
        onClose();
      }
    } catch (err: any) {
      setError(err.response?.data?.detail || 'An error occurred. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-modal-overlay">
      <div className="auth-modal-content">
        <button className="auth-modal-close" onClick={onClose}>&times;</button>
        <h2>{isLogin ? 'Login' : 'Sign Up'}</h2>
        
        {error && <div className="auth-error">{error}</div>}
        
        <form onSubmit={handleSubmit}>
          <div className="form-group">
            <label>Username</label>
            <input 
              type="text" 
              value={username} 
              onChange={(e) => setUsername(e.target.value)} 
              required 
            />
          </div>
          
          <div className="form-group">
            <label>Password</label>
            <input 
              type="password" 
              value={password} 
              onChange={(e) => setPassword(e.target.value)} 
              required 
            />
          </div>

          {!isLogin && (
            <>
              <div className="form-group">
                <label>Email</label>
                <input 
                  type="email" 
                  value={email} 
                  onChange={(e) => setEmail(e.target.value)} 
                  required 
                />
              </div>
              <div className="form-group">
                <label>Software Background</label>
                <select 
                  value={softwareBackground} 
                  onChange={(e) => setSoftwareBackground(e.target.value)}
                  required
                >
                  <option value="">Select...</option>
                  <option value="Beginner">Beginner (No coding exp)</option>
                  <option value="Intermediate">Intermediate (Python/C++ basics)</option>
                  <option value="Advanced">Advanced (Full stack/Systems)</option>
                </select>
              </div>
              <div className="form-group">
                <label>Hardware Background</label>
                <select 
                  value={hardwareBackground} 
                  onChange={(e) => setHardwareBackground(e.target.value)}
                  required
                >
                  <option value="">Select...</option>
                  <option value="None">None</option>
                  <option value="Arduino/RPi">Arduino/RPi Hobbyist</option>
                  <option value="Embedded Systems">Embedded Systems Engineer</option>
                  <option value="Robotics">Robotics Expert</option>
                </select>
              </div>
            </>
          )}

          <button type="submit" disabled={loading} className="auth-submit-btn">
            {loading ? 'Processing...' : (isLogin ? 'Login' : 'Sign Up')}
          </button>
        </form>

        <p className="auth-toggle-text">
          {isLogin ? "Don't have an account? " : "Already have an account? "}
          <span onClick={() => setIsLogin(!isLogin)}>
            {isLogin ? 'Sign Up' : 'Login'}
          </span>
        </p>
      </div>
    </div>
  );
};

export default AuthModal;
