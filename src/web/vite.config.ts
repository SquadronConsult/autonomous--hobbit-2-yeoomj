// @vitejs/plugin-react version: ^3.1.0
// vite version: ^4.3.0
import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import path from 'path';

/**
 * Helper function to resolve TypeScript path aliases for Vite configuration
 * @param paths Record of TypeScript path aliases from tsconfig.json
 * @returns Record of resolved Vite aliases
 */
function resolveAlias(paths: Record<string, string>): Record<string, string> {
  const aliases: Record<string, string> = {};
  
  // Define standard project aliases
  const baseAliases = {
    '@': '/src',
    '@components': '/src/components',
    '@pages': '/src/pages',
    '@hooks': '/src/hooks',
    '@services': '/src/services',
    '@utils': '/src/utils',
    '@contexts': '/src/contexts',
    '@interfaces': '/src/interfaces',
    '@constants': '/src/constants',
    '@assets': '/src/assets',
    '@styles': '/src/styles'
  };

  // Resolve all aliases to absolute paths
  Object.entries(baseAliases).forEach(([key, value]) => {
    aliases[key] = path.resolve(__dirname, value);
  });

  return aliases;
}

// Vite configuration
export default defineConfig({
  plugins: [
    react({
      // Enable Fast Refresh for development
      fastRefresh: true,
      // Use the automatic JSX runtime
      jsxRuntime: 'automatic'
    })
  ],

  // Development server configuration
  server: {
    port: 3000,
    host: true,
    proxy: {
      // Proxy API requests to backend server
      '/api': {
        target: 'http://localhost:8000',
        changeOrigin: true
      },
      // Proxy WebSocket connections
      '/ws': {
        target: 'ws://localhost:8000',
        ws: true
      }
    }
  },

  // Build configuration
  build: {
    outDir: 'dist',
    sourcemap: true,
    minify: 'esbuild',
    // Target modern browsers as per requirements
    target: ['chrome90', 'firefox88', 'safari14'],
    // Increase chunk size warning limit for larger applications
    chunkSizeWarningLimit: 1000
  },

  // Path resolution configuration
  resolve: {
    alias: resolveAlias({})
  },

  // CSS configuration
  css: {
    modules: {
      // Use camelCase for CSS modules class names
      localsConvention: 'camelCase'
    },
    preprocessorOptions: {
      scss: {
        // Import global variables in all SCSS files
        additionalData: '@import "@styles/variables.css";'
      }
    }
  },

  // Test configuration for Vitest
  test: {
    globals: true,
    environment: 'jsdom',
    setupFiles: ['./tests/setup.ts']
  }
});