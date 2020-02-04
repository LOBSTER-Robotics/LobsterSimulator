import getopt, sys
import Main

def main(argv):
    args = argv[1:]
    unixOptions = "ho:v"
    gnuOptions = ["help", "gui=", "tcp="]
    try:
        arguments, values = getopt.getopt(args, unixOptions, gnuOptions)
    except getopt.error as e:
        print(str(e))
        sys.exit(2)

    arguments = dict(arguments)

    gui = True
    if arguments['--gui'] is not None and arguments['--gui'] == "false":
        gui = False

    tcp = True
    if arguments['--tcp'] is not None and arguments['--tcp'] == "false":
        tcp = False

    Main.main(gui=gui, tcp=tcp)




if __name__ == '__main__':
    main(sys.argv)

