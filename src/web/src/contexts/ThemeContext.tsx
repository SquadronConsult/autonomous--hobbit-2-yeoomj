import React from 'react'; // ^18.0.0
import { ThemeMode, lightTheme, darkTheme } from '../constants/theme';

// Storage key for persisting theme preference
const STORAGE_KEY = 'theme-preference';

// Media queries for system preferences
const MEDIA_QUERY = '(prefers-color-scheme: dark)';
const REDUCED_MOTION_QUERY = '(prefers-reduced-motion: reduce)';

// Theme transition configuration
const THEME_TRANSITION_CLASS = 'theme-transition';
const TRANSITION_DURATION = 150; // milliseconds

// Theme context interface
interface ThemeContextType {
  theme: ThemeMode;
  setTheme: (theme: ThemeMode) => void;
  isDarkMode: boolean;
  toggleTheme: () => void;
  isSystemPreference: boolean;
  prefersReducedMotion: boolean;
}

// Create theme context with null initial value
const ThemeContext = React.createContext<ThemeContextType | null>(null);

// Props interface for ThemeProvider
interface ThemeProviderProps {
  children: React.ReactNode;
}

/**
 * Theme Provider component that manages theme state and system preferences
 */
export const ThemeProvider: React.FC<ThemeProviderProps> = ({ children }) => {
  // Get initial theme from localStorage or default to system
  const getInitialTheme = (): ThemeMode => {
    const stored = localStorage.getItem(STORAGE_KEY);
    return stored ? (stored as ThemeMode) : ThemeMode.SYSTEM;
  };

  // State management
  const [theme, setThemeState] = React.useState<ThemeMode>(getInitialTheme);
  const [isDarkMode, setIsDarkMode] = React.useState<boolean>(
    window.matchMedia(MEDIA_QUERY).matches
  );
  const [prefersReducedMotion, setPrefersReducedMotion] = React.useState<boolean>(
    window.matchMedia(REDUCED_MOTION_QUERY).matches
  );

  // Memoized system preference state
  const isSystemPreference = React.useMemo(
    () => theme === ThemeMode.SYSTEM,
    [theme]
  );

  // Apply theme to document root
  const applyTheme = React.useCallback((dark: boolean) => {
    const root = document.documentElement;
    const themeColors = dark ? darkTheme : lightTheme;

    // Apply theme colors to CSS variables
    Object.entries(themeColors).forEach(([category, values]) => {
      Object.entries(values).forEach(([key, value]) => {
        root.style.setProperty(
          `--theme-${category}-${key}`,
          value.toString()
        );
      });
    });

    // Update color-scheme
    root.style.colorScheme = dark ? 'dark' : 'light';
    setIsDarkMode(dark);
  }, []);

  // Theme transition handler
  const handleThemeTransition = React.useCallback((callback: () => void) => {
    if (prefersReducedMotion) {
      callback();
      return;
    }

    document.documentElement.classList.add(THEME_TRANSITION_CLASS);
    callback();

    setTimeout(() => {
      document.documentElement.classList.remove(THEME_TRANSITION_CLASS);
    }, TRANSITION_DURATION);
  }, [prefersReducedMotion]);

  // Theme setter with transition
  const setTheme = React.useCallback((newTheme: ThemeMode) => {
    if (!Object.values(ThemeMode).includes(newTheme)) {
      console.error(`Invalid theme value: ${newTheme}`);
      return;
    }

    handleThemeTransition(() => {
      setThemeState(newTheme);
      localStorage.setItem(STORAGE_KEY, newTheme);

      if (newTheme === ThemeMode.SYSTEM) {
        applyTheme(window.matchMedia(MEDIA_QUERY).matches);
      } else {
        applyTheme(newTheme === ThemeMode.DARK);
      }
    });
  }, [handleThemeTransition, applyTheme]);

  // Theme toggle handler
  const toggleTheme = React.useCallback(() => {
    const newTheme = theme === ThemeMode.LIGHT ? ThemeMode.DARK : ThemeMode.LIGHT;
    setTheme(newTheme);
  }, [theme, setTheme]);

  // System preference change handlers
  React.useEffect(() => {
    const darkModeQuery = window.matchMedia(MEDIA_QUERY);
    const reducedMotionQuery = window.matchMedia(REDUCED_MOTION_QUERY);

    const handleDarkModeChange = (e: MediaQueryListEvent) => {
      if (theme === ThemeMode.SYSTEM) {
        handleThemeTransition(() => applyTheme(e.matches));
      }
    };

    const handleReducedMotionChange = (e: MediaQueryListEvent) => {
      setPrefersReducedMotion(e.matches);
    };

    // Add listeners
    darkModeQuery.addEventListener('change', handleDarkModeChange);
    reducedMotionQuery.addEventListener('change', handleReducedMotionChange);

    // Initial theme application
    if (theme === ThemeMode.SYSTEM) {
      applyTheme(darkModeQuery.matches);
    } else {
      applyTheme(theme === ThemeMode.DARK);
    }

    // Cleanup listeners
    return () => {
      darkModeQuery.removeEventListener('change', handleDarkModeChange);
      reducedMotionQuery.removeEventListener('change', handleReducedMotionChange);
    };
  }, [theme, applyTheme, handleThemeTransition]);

  // Memoized context value
  const contextValue = React.useMemo(
    () => ({
      theme,
      setTheme,
      isDarkMode,
      toggleTheme,
      isSystemPreference,
      prefersReducedMotion,
    }),
    [theme, setTheme, isDarkMode, toggleTheme, isSystemPreference, prefersReducedMotion]
  );

  return (
    <ThemeContext.Provider value={contextValue}>
      {children}
    </ThemeContext.Provider>
  );
};

/**
 * Custom hook for accessing theme context with type safety
 */
export const useTheme = (): ThemeContextType => {
  const context = React.useContext(ThemeContext);
  
  if (!context) {
    throw new Error(
      'useTheme must be used within a ThemeProvider. ' +
      'Wrap a parent component in <ThemeProvider> to fix this error.'
    );
  }
  
  return context;
};

export default ThemeProvider;