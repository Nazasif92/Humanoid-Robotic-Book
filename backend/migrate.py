#!/usr/bin/env python3
"""Database migration runner for RAG Chatbot.

This script applies SQL migrations to the Neon PostgreSQL database.
Migrations are stored in ./migrations/ directory and numbered sequentially.

Usage:
    python migrate.py --up         # Apply pending migrations
    python migrate.py --down       # Rollback last migration
    python migrate.py --status     # Show migration status
    python migrate.py --reset      # Reset all migrations (dev only)
"""

import asyncio
import argparse
import logging
import sys
from pathlib import Path
from datetime import datetime
import asyncpg
from typing import List, Tuple

sys.path.insert(0, str(Path(__file__).parent))

from app.config import settings

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


class MigrationRunner:
    """Manages database migrations."""

    def __init__(self):
        """Initialize migration runner."""
        self.migrations_dir = Path(__file__).parent / "migrations"
        self.connection_string = settings.neon_connection_string

    async def connect(self) -> asyncpg.Connection:
        """Connect to database."""
        try:
            conn = await asyncpg.connect(self.connection_string)
            logger.info("Connected to database")
            return conn
        except Exception as e:
            logger.error(f"Failed to connect to database: {e}")
            raise

    async def init_migrations_table(self, conn: asyncpg.Connection) -> None:
        """Create migrations tracking table if it doesn't exist."""
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS schema_migrations (
                id SERIAL PRIMARY KEY,
                version VARCHAR(255) NOT NULL UNIQUE,
                description VARCHAR(500),
                installed_on TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                execution_time_ms INTEGER
            );
        """)
        logger.info("Migrations table ready")

    async def get_applied_migrations(self, conn: asyncpg.Connection) -> List[str]:
        """Get list of applied migrations."""
        rows = await conn.fetch("""
            SELECT version FROM schema_migrations
            ORDER BY version ASC;
        """)
        return [row["version"] for row in rows]

    async def get_pending_migrations(self) -> List[Tuple[str, str]]:
        """Get list of pending migrations.

        Returns:
            List of (version, filepath) tuples
        """
        if not self.migrations_dir.exists():
            logger.warning(f"Migrations directory not found: {self.migrations_dir}")
            return []

        pending = []
        for migration_file in sorted(self.migrations_dir.glob("*.sql")):
            version = migration_file.stem
            pending.append((version, migration_file))

        return pending

    async def apply_migration(
        self,
        conn: asyncpg.Connection,
        version: str,
        filepath: Path,
    ) -> None:
        """Apply a single migration.

        Args:
            conn: Database connection
            version: Migration version/name
            filepath: Path to SQL file
        """
        logger.info(f"Applying migration: {version}")

        try:
            # Read SQL file
            with open(filepath, "r") as f:
                sql = f.read()

            # Execute migration in transaction
            start_time = datetime.now()
            async with conn.transaction():
                await conn.execute(sql)

            # Record migration
            exec_time = (datetime.now() - start_time).total_seconds() * 1000
            await conn.execute(
                """
                INSERT INTO schema_migrations (version, description, execution_time_ms)
                VALUES ($1, $2, $3);
                """,
                version,
                f"Automated migration from {filepath.name}",
                int(exec_time),
            )

            logger.info(f"✓ Applied {version} ({exec_time:.0f}ms)")

        except Exception as e:
            logger.error(f"✗ Migration {version} failed: {e}")
            raise

    async def migrate_up(self) -> None:
        """Apply pending migrations."""
        conn = await self.connect()
        try:
            await self.init_migrations_table(conn)

            applied = await self.get_applied_migrations(conn)
            pending = await self.get_pending_migrations()

            pending_to_apply = [
                (v, f) for v, f in pending if v not in applied
            ]

            if not pending_to_apply:
                logger.info("✓ All migrations applied")
                return

            logger.info(f"Found {len(pending_to_apply)} pending migration(s)")

            for version, filepath in pending_to_apply:
                await self.apply_migration(conn, version, filepath)

            logger.info("✓ All pending migrations applied successfully")

        finally:
            await conn.close()

    async def migrate_status(self) -> None:
        """Show migration status."""
        conn = await self.connect()
        try:
            await self.init_migrations_table(conn)

            applied = await self.get_applied_migrations(conn)
            all_migrations = await self.get_pending_migrations()

            print("\n" + "=" * 60)
            print("MIGRATION STATUS")
            print("=" * 60)

            if applied:
                print(f"\nApplied ({len(applied)}):")
                for version in applied:
                    print(f"  ✓ {version}")

            pending = [v for v, _ in all_migrations if v not in applied]
            if pending:
                print(f"\nPending ({len(pending)}):")
                for version in pending:
                    print(f"  ○ {version}")
            else:
                print("\n✓ No pending migrations")

            print("=" * 60 + "\n")

        finally:
            await conn.close()

    async def migrate_reset(self) -> None:
        """Reset all migrations (development only)."""
        if settings.backend_host != "localhost":
            logger.error("Reset is only allowed in development mode!")
            sys.exit(1)

        conn = await self.connect()
        try:
            logger.warning("Resetting all migrations...")
            await conn.execute("DROP TABLE IF EXISTS schema_migrations CASCADE;")
            logger.info("✓ Migrations reset")
        finally:
            await conn.close()


async def main() -> None:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Database migration runner for RAG Chatbot"
    )
    parser.add_argument(
        "--up",
        action="store_true",
        help="Apply pending migrations",
    )
    parser.add_argument(
        "--status",
        action="store_true",
        help="Show migration status",
    )
    parser.add_argument(
        "--reset",
        action="store_true",
        help="Reset all migrations (dev only)",
    )

    args = parser.parse_args()

    runner = MigrationRunner()

    try:
        if args.up:
            await runner.migrate_up()
        elif args.status:
            await runner.migrate_status()
        elif args.reset:
            await runner.migrate_reset()
        else:
            # Default to status
            await runner.migrate_status()

    except Exception as e:
        logger.error(f"Migration failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
