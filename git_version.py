Import("env")
import subprocess
import os

def get_git_version():
    """Get version from git describe or fall back to 'dev'"""
    try:
        # Try git describe first (for tagged releases)
        version = subprocess.check_output(
            ["git", "describe", "--tags", "--always", "--dirty"],
            stderr=subprocess.DEVNULL,
            cwd=env.get("PROJECT_DIR", os.getcwd())
        ).decode().strip()
        return version
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "dev"

def get_git_commit():
    """Get short commit hash"""
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            stderr=subprocess.DEVNULL,
            cwd=env.get("PROJECT_DIR", os.getcwd())
        ).decode().strip()
        return commit
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "unknown"

def get_build_time():
    """Get build timestamp"""
    from datetime import datetime
    return datetime.now().strftime("%Y-%m-%d %H:%M")

# Get version info
git_version = get_git_version()
git_commit = get_git_commit()
build_time = get_build_time()

print(f"* Git Version: {git_version}")
print(f"* Git Commit:  {git_commit}")
print(f"* Build Time:  {build_time}")

# Add build flags
env.Append(CPPDEFINES=[
    ("GIT_VERSION", f'\\"{git_version}\\"'),
    ("GIT_COMMIT", f'\\"{git_commit}\\"'),
    ("BUILD_TIME", f'\\"{build_time}\\"'),
])
