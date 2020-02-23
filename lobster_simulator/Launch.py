import getopt, sys
from lobster_simulator import Main


def main(argv):
    args = argv[1:]
    unixOptions = "h"
    gnuOptions = ["help", "gui=", "tcp="]
    try:
        arguments, values = getopt.getopt(args, unixOptions, gnuOptions)
    except getopt.error as e:
        print(str(e))
        sys.exit(2)

    arguments = dict(arguments)

    if '-h' in arguments or '--help' in arguments:
        print("Unfortunately there is no help yet...")
        return

    gui = True
    if '--gui' in arguments and arguments['--gui'] == "false":
        gui = False

    tcp = True
    if '--tcp' in arguments and arguments['--tcp'] == "false":
        tcp = False

    Main.main(gui=gui, tcp=tcp)


if __name__ == '__main__':
    main(sys.argv)

