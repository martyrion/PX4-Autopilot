#!/usr/bin/env python3
"""
Simple Parameter File Updater - FIXED VERSION
Updates parameters by reading entire file as text and using string replacement
"""

import os
import time
import subprocess
import signal
from pathlib import Path
import itertools
import numpy as np
from datetime import datetime
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class SimpleParamUpdater:
    def __init__(self, config):
        self.px4_dir = Path(config['px4_directory'])
        self.replay_params_file = Path(config['replay_params_file'])
        self.output_dir = Path(config['output_directory'])

        # Create output directory
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.run_count = 0

        # Read the original file content as a template
        with open(self.replay_params_file, 'r') as f:
            self.original_content = f.read()

        logger.info(f"✅ Loaded parameter file with {len(self.original_content.splitlines())} lines")

    def update_params_file(self, new_params):
        """Update parameters using string replacement - MUCH SAFER"""

        # Start with original content
        updated_content = self.original_content

        # Track updates
        updated_count = 0

        # Update each parameter using string replacement
        for param_name, new_value in new_params.items():
            # Find the line with this parameter
            lines = updated_content.splitlines()

            for i, line in enumerate(lines):
                if line.startswith(f"{param_name} "):
                    # Extract the old value
                    parts = line.split(' ', 1)
                    if len(parts) == 2:
                        old_value = parts[1]
                        # Replace the entire line
                        new_line = f"{param_name} {new_value}"
                        lines[i] = new_line
                        updated_count += 1
                        logger.info(f"  ✅ Updated {param_name}: {old_value} → {new_value}")
                        break
            else:
                logger.warning(f"⚠️ Parameter not found: {param_name}")

        # Join lines back with proper newlines
        updated_content = '\n'.join(lines)

        # Write to file
        try:
            with open(self.replay_params_file, 'w') as f:
                f.write(updated_content)

            # Verify by reading back
            with open(self.replay_params_file, 'r') as f:
                verify_content = f.read()

            if len(verify_content.splitlines()) != len(self.original_content.splitlines()):
                logger.error(f"❌ Line count mismatch! Expected {len(self.original_content.splitlines())}, got {len(verify_content.splitlines())}")
                return 0

            # Check for corruption
            if '\\n' in verify_content:
                logger.error("❌ File contains literal \\n characters - corruption detected!")
                return 0

            logger.info(f"📝 Successfully updated {updated_count} parameters")
            return updated_count

        except Exception as e:
            logger.error(f"❌ Error writing file: {e}")
            return 0

    def kill_existing_px4(self):
        """Kill any existing PX4 processes"""
        logger.info("🔍 Checking for existing PX4 processes...")

        try:
            subprocess.run(['pkill', '-f', 'px4'], check=False)
            time.sleep(2)
            subprocess.run(['pkill', '-f', 'px4_sitl'], check=False)
            time.sleep(1)
            logger.info("🧹 Cleaned up existing processes")
        except Exception as e:
            logger.warning(f"⚠️ Error cleaning processes: {e}")

    def run_px4_replay(self):
        """Run PX4 replay"""
        logger.info("🚁 Starting PX4 replay...")

        try:
            # Kill existing processes first
            self.kill_existing_px4()

            # Run PX4
            process = subprocess.Popen(
                ['make', 'px4_sitl', 'none'],
                cwd=self.px4_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid
            )

            # Wait 15 seconds
            logger.info("⏱️ Waiting 20 seconds for replay to complete...")
            time.sleep(20)

            # Kill process
            logger.info("🛑 Terminating PX4...")
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
                time.sleep(2)
                if process.poll() is None:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    time.sleep(2)
                if process.poll() is None:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass

            # Additional cleanup
            self.kill_existing_px4()

            logger.info("✅ PX4 replay completed")
            return True

        except Exception as e:
            logger.error(f"❌ Error running PX4 replay: {e}")
            self.kill_existing_px4()
            return False

    def save_run_info(self, params, success):
        """Save run information"""
        run_info_file = self.output_dir / f"run_{self.run_count:03d}_info.txt"

        with open(run_info_file, 'w') as f:
            f.write(f"Run {self.run_count} Information\\n")
            f.write(f"========================\\n")
            f.write(f"Timestamp: {datetime.now()}\\n")
            f.write(f"Success: {success}\\n")
            f.write(f"\\nParameters tested:\\n")
            for param, value in params.items():
                f.write(f"  {param}: {value}\\n")

    def generate_parameter_combinations(self, param_ranges):
        """Generate parameter combinations"""
        param_lists = {}
        for param, values in param_ranges.items():
            if isinstance(values, (list, tuple, np.ndarray)):
                param_lists[param] = values
            else:
                param_lists[param] = [values]

        keys = list(param_lists.keys())
        values = list(param_lists.values())

        combinations = []
        for combo in itertools.product(*values):
            param_dict = dict(zip(keys, combo))
            combinations.append(param_dict)

        logger.info(f"📊 Generated {len(combinations)} parameter combinations")
        return combinations

    def run_parameter_sweep(self, param_ranges):
        """Run parameter sweep"""
        logger.info("🚀 Starting parameter sweep")

        combinations = self.generate_parameter_combinations(param_ranges)
        success_count = 0

        try:
            for i, params in enumerate(combinations, 1):
                self.run_count = i
                logger.info(f"\\n🔄 Run {i}/{len(combinations)}")
                logger.info(f"📋 Testing parameters: {params}")

                try:
                    # Update parameters
                    updated_count = self.update_params_file(params)
                    if updated_count == 0:
                        logger.error("❌ Failed to update parameters, skipping run")
                        continue

                    # Run PX4
                    success = self.run_px4_replay()

                    # Save info
                    self.save_run_info(params, success)

                    if success:
                        success_count += 1
                        logger.info(f"✅ Run {i} completed successfully")
                    else:
                        logger.warning(f"⚠️ Run {i} had issues")

                    time.sleep(3)  # Brief pause

                except KeyboardInterrupt:
                    logger.info("⏹️ Parameter sweep interrupted by user")
                    break
                except Exception as e:
                    logger.error(f"❌ Error in run {i}: {e}")
                    continue

        finally:
            # Restore original file
            logger.info("🔄 Restoring original parameter file...")
            with open(self.replay_params_file, 'w') as f:
                f.write(self.original_content)

        logger.info(f"\\n🏁 Parameter sweep complete!")
        logger.info(f"✅ Successful runs: {success_count}/{len(combinations)}")


# Parameter ranges for DR-13 fine-tuning
def get_dr13_micro_sweep():
    """Micro sweep around dr-13 values"""
    return {
        'EKF2_ACC_B_NOISE': [0.0048, 0.005, 0.0052],
        'EKF2_GYR_B_NOISE': [0.00029, 0.0003, 0.00031],
        'EKF2_ACC_NOISE': [0.34, 0.35, 0.36],
        'EKF2_GYR_NOISE': [0.0175, 0.018, 0.0185],
        'EKF2_NOAID_NOISE': [0.48, 0.5, 0.52],
    }

def get_single_param_test():
    """Test just one parameter"""
    return {
        'EKF2_ACC_B_NOISE': [0.0048, 0.005, 0.0052],
    }


def main():
    """Main function"""
    config = {
        'px4_directory': '/home/dimitris/Desktop/PX4-Autopilot',
        'replay_params_file': '/home/dimitris/Desktop/PX4-Autopilot/build/px4_sitl_default_replay/rootfs/replay_params.txt',
        'output_directory': './px4_param_sweep_results'
    }

    print("🚁 Simple Parameter Updater - FIXED VERSION")
    print("=" * 60)

    # Verify file exists
    if not Path(config['replay_params_file']).exists():
        print(f"❌ Parameter file not found: {config['replay_params_file']}")
        return

    print("Choose test:")
    print("1. Single parameter test (3 combinations) - RECOMMENDED FIRST")
    print("2. DR-13 micro sweep (243 combinations)")

    choice = input("Enter choice (1-2): ").strip()

    if choice == '1':
        param_ranges = get_single_param_test()
        print("🎯 Testing single parameter (quick test)")
    elif choice == '2':
        param_ranges = get_dr13_micro_sweep()
        print("🎯 DR-13 micro sweep")
    else:
        param_ranges = get_single_param_test()
        print("🎯 Default: single parameter test")

    # Initialize updater
    updater = SimpleParamUpdater(config)

    # Calculate combinations
    combinations = updater.generate_parameter_combinations(param_ranges)

    print(f"\\n📊 Will run {len(combinations)} combinations")
    print(f"⏱️ Estimated time: {len(combinations) * 0.5:.1f} minutes")

    # Confirm
    confirm = input("\\nProceed? (y/N): ").strip().lower()
    if confirm != 'y':
        print("Cancelled")
        return

    # Run sweep
    updater.run_parameter_sweep(param_ranges)

    print(f"\\n✅ Done! Results in: {config['output_directory']}")


if __name__ == "__main__":
    main()
