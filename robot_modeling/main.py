
import argparse

from swerve.swerve_id import swerve_sysid, SWERVE_MODULE_ID
from swerve.swerve_ctrl import swerve_control, SWERVE_MODULE_CTRL

if __name__ == "__main__":
    # Set up argument parsing
    parser = argparse.ArgumentParser(prog="Team OKC System ID", description="Perform system ID on Team 2718's robot")

    # Add arguments
    parser.add_argument("module")

    # Parse arguments
    args = parser.parse_args()

    # Do system ID based on the arguments
    if args.module == SWERVE_MODULE_ID:
        # Swerve module sysID selected
        swerve_sysid()
    elif args.module == SWERVE_MODULE_CTRL:
        # Swerve module control design selected
        swerve_control()
    else:
        # default: return an error
        print(f"The specified module '{args.module}' has not been added to main.py yet. Please add it first")

