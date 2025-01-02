# Build stage
FROM node:16-alpine AS builder

# Add build dependencies and security patches
RUN apk add --no-cache \
    python3 \
    make \
    g++ \
    git \
    && apk upgrade --no-cache

# Set working directory
WORKDIR /app

# Set build arguments and environment variables
ARG APP_VERSION
ENV NODE_ENV=production \
    VITE_APP_VERSION=${APP_VERSION}

# Copy package files first for better cache utilization
COPY package*.json ./
COPY tsconfig*.json ./
COPY vite.config.ts ./

# Install dependencies with exact versions and clean cache
RUN npm ci --production=false \
    && npm cache clean --force

# Copy source code
COPY . .

# Build application
RUN npm run build \
    && npm prune --production

# Production stage
FROM nginx:1.23-alpine

# Add security patches and remove unnecessary tools
RUN apk upgrade --no-cache \
    && apk add --no-cache curl \
    && rm -rf /usr/local/sbin/* \
    && rm -rf /usr/local/bin/* \
    && rm -rf /usr/sbin/* \
    && rm -rf /usr/bin/* \
    && rm -rf /sbin/* \
    && rm -rf /bin/* \
    && apk add --no-cache wget

# Create nginx user and group with restricted permissions
RUN addgroup -g 101 -S nginx \
    && adduser -S -D -H -u 101 -h /var/cache/nginx -s /sbin/nologin -G nginx -g nginx nginx

# Copy custom nginx configuration
COPY nginx.conf /etc/nginx/nginx.conf
COPY nginx.default.conf /etc/nginx/conf.d/default.conf

# Copy built assets from builder stage
COPY --from=builder --chown=nginx:nginx /app/dist /usr/share/nginx/html

# Set correct permissions
RUN chmod -R 555 /usr/share/nginx/html \
    && chown -R nginx:nginx /usr/share/nginx/html \
    && chmod -R 555 /etc/nginx \
    && chown -R nginx:nginx /etc/nginx \
    && mkdir -p /var/cache/nginx /var/run \
    && chmod -R 555 /var/cache/nginx \
    && chown -R nginx:nginx /var/cache/nginx /var/run

# Configure security headers
ENV SECURITY_HEADERS="add_header X-Frame-Options DENY; \
    add_header X-Content-Type-Options nosniff; \
    add_header X-XSS-Protection \"1; mode=block\"; \
    add_header Content-Security-Policy \"default-src 'self'; script-src 'self' 'unsafe-inline' 'unsafe-eval'; style-src 'self' 'unsafe-inline'; img-src 'self' data: https:; font-src 'self' data:; connect-src 'self' https: wss:;\"; \
    add_header Strict-Transport-Security \"max-age=31536000; includeSubDomains\" always;"

# Set working directory
WORKDIR /usr/share/nginx/html

# Switch to non-root user
USER nginx

# Expose port
EXPOSE 80

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD wget -q -O - http://localhost:80/health || exit 1

# Add metadata labels
LABEL maintainer="Agricultural Management System Team" \
    description="Web dashboard for agricultural management system" \
    version=${APP_VERSION}

# Use read-only root filesystem
ENV NGINX_ENTRYPOINT_QUIET_LOGS=1
CMD ["nginx", "-g", "daemon off;"]

# Set read-only root filesystem
VOLUME ["/tmp", "/var/run", "/var/cache/nginx"]