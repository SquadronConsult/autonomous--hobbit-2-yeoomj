# Agricultural Management System Web Dashboard

## Overview

The Agricultural Management System Web Dashboard is a modern, accessible, and responsive web application built with React 18 and Material Design 3.0. It provides real-time monitoring, mission management, and analytics visualization for autonomous agricultural operations.

### Key Features
- Real-time fleet monitoring and telemetry visualization
- Mission planning and management interface
- Analytics dashboard with performance metrics
- Interactive coverage maps and tracking
- WCAG 2.1 Level AA compliant accessibility
- Internationalization with RTL support

## Prerequisites

- Node.js >= 16.0.0
- npm >= 8.0.0
- Modern evergreen browser (Chrome 90+, Firefox 88+, Safari 14+)
- Minimum screen resolution: 1024x768
- Hardware acceleration recommended for map rendering

## Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd src/web
```

2. Install dependencies:
```bash
npm install
```

3. Copy environment configuration:
```bash
cp .env.example .env
```

4. Configure environment variables according to your deployment

## Development

### Starting Development Server
```bash
npm run dev
```
Server will start at http://localhost:3000 with hot reload enabled

### Code Style
- ESLint configuration enforces Airbnb style guide
- Prettier handles code formatting
- TypeScript strict mode enabled
- Component-specific styles using CSS Modules

### Git Workflow
1. Create feature branch from development
2. Follow conventional commits specification
3. Submit PR for review
4. Automated tests must pass
5. Require accessibility compliance check

## Building

### Production Build
```bash
npm run build
```

Build output will be in `dist/` directory with the following optimizations:
- Code splitting and lazy loading
- Asset optimization and compression
- Tree shaking for minimal bundle size
- Source maps generation

### Build Analysis
```bash
npm run build:analyze
```

## Testing

### Running Tests
```bash
# Run all tests
npm test

# Run unit tests
npm run test:unit

# Run integration tests
npm run test:integration

# Run accessibility tests
npm run test:a11y
```

### Testing Stack
- Jest for unit testing
- React Testing Library for component testing
- Cypress for E2E testing
- axe-core for accessibility testing

## Project Structure

```
src/web/
├── public/          # Static assets
├── src/
│   ├── components/  # Reusable UI components
│   ├── hooks/       # Custom React hooks
│   ├── pages/       # Route components
│   ├── services/    # API and external services
│   ├── store/       # State management
│   ├── styles/      # Global styles and themes
│   ├── types/       # TypeScript definitions
│   └── utils/       # Helper functions
├── tests/           # Test suites
└── config/          # Build configurations
```

## Environment Variables

Required environment variables:
```
VITE_API_URL=        # Backend API endpoint
VITE_MAPBOX_TOKEN=   # MapBox API token
VITE_AUTH_DOMAIN=    # Authentication domain
VITE_ENVIRONMENT=    # deployment environment
```

## Browser Support

| Browser | Minimum Version |
|---------|----------------|
| Chrome  | 90+            |
| Firefox | 88+            |
| Safari  | 14+            |
| Edge    | 90+            |

## Contributing

1. Review contribution guidelines
2. Set up development environment
3. Follow code style guidelines
4. Write tests for new features
5. Ensure accessibility compliance
6. Submit PR with detailed description

## Accessibility

### WCAG 2.1 Level AA Compliance
- Keyboard navigation support
- ARIA labels implementation
- Color contrast requirements
- Focus management
- Screen reader compatibility
- Responsive text sizing

### Testing Requirements
- Automated accessibility testing
- Manual screen reader testing
- Keyboard navigation validation
- Color contrast verification

## Internationalization

### Supported Features
- RTL layout support
- Dynamic text direction
- Locale-specific formatting
- Translation management
- Cultural adaptation

### Adding New Languages
1. Add locale file in `src/locales/`
2. Update language selector
3. Test RTL if applicable
4. Verify translations
5. Update documentation

## Technologies

- React 18.2.0 - UI framework
- TypeScript 5.0.4 - Type safety
- Material UI 5.12.0 - Component library
- Vite 4.3.0 - Build tool
- React Router 6.11.0 - Routing
- React Query 4.29.0 - Data fetching
- Redux Toolkit 1.9.5 - State management
- MapBox GL 2.15.0 - Map visualization
- Jest 29.5.0 - Testing framework
- Cypress 12.11.0 - E2E testing

## License

Copyright © 2024 Agricultural Management System