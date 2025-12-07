"""Monitoring and observability for RAG Chatbot.

Provides:
- Structured logging with context
- Request/response metrics collection
- Health check aggregation
- Performance monitoring utilities
"""

import logging
import time
from typing import Any, Dict, Optional, Callable
from functools import wraps
from datetime import datetime
import json

from .config import settings

# Configure JSON structured logging
class JSONFormatter(logging.Formatter):
    """Format logs as JSON for better parsing."""

    def format(self, record: logging.LogRecord) -> str:
        """Format log record as JSON."""
        log_obj = {
            "timestamp": datetime.utcnow().isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
            "module": record.module,
            "function": record.funcName,
            "line": record.lineno,
        }

        # Add exception info if present
        if record.exc_info:
            log_obj["exception"] = self.formatException(record.exc_info)

        # Add extra fields if present
        if hasattr(record, "extra_fields"):
            log_obj.update(record.extra_fields)

        return json.dumps(log_obj)


def setup_logging(level: str = "INFO") -> logging.Logger:
    """Setup structured logging.

    Args:
        level: Logging level (DEBUG, INFO, WARNING, ERROR)

    Returns:
        Configured logger
    """
    logger = logging.getLogger("rag_chatbot")
    logger.setLevel(level)

    # Remove existing handlers
    logger.handlers.clear()

    # Console handler with JSON formatting
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(JSONFormatter())
    logger.addHandler(console_handler)

    return logger


# Global logger
logger = setup_logging(settings.log_level)


class MetricsCollector:
    """Collect and aggregate performance metrics."""

    def __init__(self):
        """Initialize metrics collector."""
        self.metrics: Dict[str, list] = {}

    def record(self, metric_name: str, value: float, tags: Optional[Dict[str, str]] = None) -> None:
        """Record a metric value.

        Args:
            metric_name: Name of metric (e.g., 'latency_ms')
            value: Metric value
            tags: Optional tags for categorization
        """
        if metric_name not in self.metrics:
            self.metrics[metric_name] = []

        entry = {
            "timestamp": time.time(),
            "value": value,
            "tags": tags or {},
        }
        self.metrics[metric_name].append(entry)

        # Keep last 1000 entries per metric (rolling window)
        if len(self.metrics[metric_name]) > 1000:
            self.metrics[metric_name].pop(0)

    def get_stats(self, metric_name: str) -> Optional[Dict[str, float]]:
        """Get statistics for a metric.

        Args:
            metric_name: Name of metric

        Returns:
            Dict with min, max, mean, count
        """
        if metric_name not in self.metrics or not self.metrics[metric_name]:
            return None

        values = [e["value"] for e in self.metrics[metric_name]]
        return {
            "count": len(values),
            "min": min(values),
            "max": max(values),
            "mean": sum(values) / len(values),
            "sum": sum(values),
        }

    def clear(self) -> None:
        """Clear all metrics."""
        self.metrics.clear()


# Global metrics collector
metrics = MetricsCollector()


def monitor_performance(metric_name: str = None):
    """Decorator to monitor function performance.

    Records execution time and logs the call.

    Args:
        metric_name: Name for the metric (defaults to function name)

    Example:
        @monitor_performance("embedding_time")
        async def embed_query(query: str):
            ...
    """
    def decorator(func: Callable) -> Callable:
        name = metric_name or f"{func.__module__}.{func.__name__}"

        @wraps(func)
        async def async_wrapper(*args, **kwargs) -> Any:
            start_time = time.time()
            try:
                result = await func(*args, **kwargs)
                duration_ms = (time.time() - start_time) * 1000
                metrics.record(name, duration_ms, {"status": "success"})
                logger.debug(
                    f"{name} completed",
                    extra={"extra_fields": {"duration_ms": duration_ms, "status": "success"}}
                )
                return result
            except Exception as e:
                duration_ms = (time.time() - start_time) * 1000
                metrics.record(name, duration_ms, {"status": "error"})
                logger.error(
                    f"{name} failed: {str(e)}",
                    extra={"extra_fields": {"duration_ms": duration_ms, "error": str(e)}}
                )
                raise

        @wraps(func)
        def sync_wrapper(*args, **kwargs) -> Any:
            start_time = time.time()
            try:
                result = func(*args, **kwargs)
                duration_ms = (time.time() - start_time) * 1000
                metrics.record(name, duration_ms, {"status": "success"})
                logger.debug(
                    f"{name} completed",
                    extra={"extra_fields": {"duration_ms": duration_ms}}
                )
                return result
            except Exception as e:
                duration_ms = (time.time() - start_time) * 1000
                metrics.record(name, duration_ms, {"status": "error"})
                logger.error(
                    f"{name} failed: {str(e)}",
                    extra={"extra_fields": {"duration_ms": duration_ms, "error": str(e)}}
                )
                raise

        # Return appropriate wrapper based on function type
        if asyncio.iscoroutinefunction(func):
            return async_wrapper
        else:
            return sync_wrapper

    return decorator


import asyncio


class HealthChecker:
    """Aggregate health checks from multiple services."""

    def __init__(self):
        """Initialize health checker."""
        self.checks: Dict[str, Callable] = {}

    def register(self, name: str, check_func: Callable) -> None:
        """Register a health check.

        Args:
            name: Name of the check
            check_func: Async function that returns bool
        """
        self.checks[name] = check_func

    async def check_all(self) -> Dict[str, bool]:
        """Run all health checks concurrently.

        Returns:
            Dict mapping check name to result (True = healthy)
        """
        results = {}
        tasks = []

        for name, check_func in self.checks.items():
            tasks.append(self._run_check(name, check_func))

        results = await asyncio.gather(*tasks, return_exceptions=False)
        return dict(zip(self.checks.keys(), results))

    async def _run_check(self, name: str, check_func: Callable) -> tuple:
        """Run a single health check with timeout.

        Args:
            name: Check name
            check_func: Check function

        Returns:
            Tuple of (name, result)
        """
        try:
            result = await asyncio.wait_for(check_func(), timeout=5.0)
            logger.debug(f"Health check '{name}': {'healthy' if result else 'unhealthy'}")
            return (name, result)
        except asyncio.TimeoutError:
            logger.warning(f"Health check '{name}' timed out")
            return (name, False)
        except Exception as e:
            logger.error(f"Health check '{name}' failed: {e}")
            return (name, False)


# Global health checker
health_checker = HealthChecker()


class RequestLogger:
    """Log HTTP requests with context."""

    @staticmethod
    def log_request(
        method: str,
        path: str,
        status_code: int,
        latency_ms: float,
        user_agent: Optional[str] = None,
        error: Optional[str] = None,
    ) -> None:
        """Log an HTTP request.

        Args:
            method: HTTP method
            path: Request path
            status_code: Response status code
            latency_ms: Response latency
            user_agent: User agent string
            error: Error message if failed
        """
        level = "error" if status_code >= 500 else "warning" if status_code >= 400 else "info"

        log_func = getattr(logger, level)
        log_func(
            f"{method} {path} {status_code}",
            extra={
                "extra_fields": {
                    "method": method,
                    "path": path,
                    "status_code": status_code,
                    "latency_ms": latency_ms,
                    "user_agent": user_agent,
                    "error": error,
                }
            }
        )

        # Record metric
        metrics.record(
            "http_latency_ms",
            latency_ms,
            {"method": method, "path": path, "status": str(status_code)}
        )


# Utility functions
def get_all_metrics() -> Dict[str, Dict[str, float]]:
    """Get all collected metrics with statistics.

    Returns:
        Dict mapping metric names to stats
    """
    result = {}
    for metric_name in metrics.metrics.keys():
        stats = metrics.get_stats(metric_name)
        if stats:
            result[metric_name] = stats
    return result


def get_service_stats() -> Dict[str, Any]:
    """Get overall service statistics.

    Returns:
        Service stats including uptime, metrics, health
    """
    return {
        "metrics": get_all_metrics(),
        "timestamp": datetime.utcnow().isoformat(),
    }
