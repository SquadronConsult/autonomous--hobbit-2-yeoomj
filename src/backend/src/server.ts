/**
 * @fileoverview Main server entry point for the Agricultural Management System
 * Implements secure HTTPS server with TLS 1.3, monitoring, and edge computing optimizations
 * @version 1.0.0
 */

import https from 'node:https';
import fs from 'node:fs';
import helmet from 'helmet';
import * as promClient from 'prom-client';
import app from './app';
import { logger } from './utils/logger';

// Environment variables with defaults
const PORT = process.env.PORT || 8443;
const NODE_ENV = process.env.NODE_ENV || 'development';
const SHUTDOWN_TIMEOUT = process.env.SHUTDOWN_TIMEOUT || 30000;

// Track active connections for graceful shutdown
let connections = new Set<https.ServerResponse>();

/**
 * Configures comprehensive Prometheus metrics collection for edge deployment
 */
async function setupMetrics(): Promise<void> {
    try {
        // Initialize default metrics with edge-specific labels
        promClient.collectDefaultMetrics({
            prefix: 'edge_',
            labels: { node: 'jetson_orin' }
        });

        // Server metrics
        const serverMetrics = {
            activeConnections: new promClient.Gauge({
                name: 'server_active_connections',
                help: 'Number of active server connections'
            }),
            requestDuration: new promClient.Histogram({
                name: 'server_request_duration_seconds',
                help: 'Request duration in seconds',
                buckets: [0.1, 0.5, 1, 2, 5]
            }),
            resourceUsage: new promClient.Gauge({
                name: 'server_resource_usage',
                help: 'Server resource usage metrics',
                labelNames: ['resource']
            })
        };

        // Update resource metrics every 15 seconds
        setInterval(() => {
            const usage = process.memoryUsage();
            serverMetrics.resourceUsage.labels('heap_used').set(usage.heapUsed);
            serverMetrics.resourceUsage.labels('heap_total').set(usage.heapTotal);
            serverMetrics.resourceUsage.labels('rss').set(usage.rss);
            serverMetrics.activeConnections.set(connections.size);
        }, 15000);

        logger.info('Metrics setup completed', {
            component: 'Server',
            operation: 'setupMetrics'
        });
    } catch (error) {
        logger.error(error as Error, {
            component: 'Server',
            operation: 'setupMetrics'
        });
        throw error;
    }
}

/**
 * Initializes and starts the HTTPS server with TLS 1.3 and security headers
 */
async function startServer(): Promise<https.Server> {
    try {
        // Load SSL certificates
        const sslOptions = {
            key: fs.readFileSync('src/backend/certs/server.key'),
            cert: fs.readFileSync('src/backend/certs/server.crt'),
            ca: fs.readFileSync('src/backend/certs/ca.crt'),
            minVersion: 'TLSv1.3',
            ciphers: [
                'TLS_AES_256_GCM_SHA384',
                'TLS_CHACHA20_POLY1305_SHA256',
                'TLS_AES_128_GCM_SHA256'
            ].join(':'),
            honorCipherOrder: true
        };

        // Configure security headers
        app.use(helmet({
            contentSecurityPolicy: {
                directives: {
                    defaultSrc: ["'self'"],
                    scriptSrc: ["'self'"],
                    styleSrc: ["'self'"],
                    imgSrc: ["'self'", 'data:', 'blob:'],
                    connectSrc: ["'self'"]
                }
            },
            hsts: {
                maxAge: 31536000,
                includeSubDomains: true,
                preload: true
            }
        }));

        // Initialize metrics
        await setupMetrics();

        // Create HTTPS server
        const server = https.createServer(sslOptions, app);

        // Configure keep-alive for edge deployment
        server.keepAliveTimeout = 120000;
        server.headersTimeout = 125000;

        // Track connections for graceful shutdown
        server.on('connection', (socket) => {
            const res = socket as unknown as https.ServerResponse;
            connections.add(res);
            socket.on('close', () => {
                connections.delete(res);
            });
        });

        // Start listening
        server.listen(PORT, () => {
            logger.info(`Server started in ${NODE_ENV} mode`, {
                component: 'Server',
                operation: 'start',
                details: { port: PORT }
            });
        });

        return server;
    } catch (error) {
        logger.error(error as Error, {
            component: 'Server',
            operation: 'startServer'
        });
        throw error;
    }
}

/**
 * Handles graceful server shutdown with connection draining
 */
async function gracefulShutdown(server: https.Server): Promise<void> {
    logger.info('Initiating graceful shutdown', {
        component: 'Server',
        operation: 'shutdown'
    });

    // Stop accepting new connections
    server.close(() => {
        logger.info('Server stopped accepting new connections', {
            component: 'Server',
            operation: 'shutdown'
        });
    });

    // Set shutdown timeout
    const shutdownTimeout = setTimeout(() => {
        logger.warn('Shutdown timeout reached, forcing exit', {
            component: 'Server',
            operation: 'shutdown'
        });
        process.exit(1);
    }, SHUTDOWN_TIMEOUT);

    try {
        // Wait for existing connections to complete
        if (connections.size > 0) {
            logger.info(`Waiting for ${connections.size} connections to close`, {
                component: 'Server',
                operation: 'shutdown'
            });

            const connectionPromises = Array.from(connections).map((res) => {
                return new Promise<void>((resolve) => {
                    res.on('finish', resolve);
                    res.destroy();
                });
            });

            await Promise.all(connectionPromises);
        }

        // Clean up resources
        connections.clear();
        clearTimeout(shutdownTimeout);

        logger.info('Graceful shutdown completed', {
            component: 'Server',
            operation: 'shutdown'
        });
        process.exit(0);
    } catch (error) {
        logger.error(error as Error, {
            component: 'Server',
            operation: 'shutdown'
        });
        process.exit(1);
    }
}

// Start server and handle shutdown signals
let server: https.Server;

startServer()
    .then((httpsServer) => {
        server = httpsServer;
        process.on('SIGTERM', () => gracefulShutdown(server));
        process.on('SIGINT', () => gracefulShutdown(server));
    })
    .catch((error) => {
        logger.error(error as Error, {
            component: 'Server',
            operation: 'startup'
        });
        process.exit(1);
    });

export default server;