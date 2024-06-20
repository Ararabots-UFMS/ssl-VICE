import argparse
import vision_client

def main(args):
    client = vision_client.Client(ip=args.ip, port=args.port)

    print(f'Binding client to {args.ip} : {args.port}')
    client.connect()

    while True:
        data = client.receive()

        # TODO apply filter to reduce noise

        if args.verbose:
            print(data)
        if args.save_log:
            # TODO save logs
            pass

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-ip', type=str, default='224.5.23.2', help='IPv4 format')
    parser.add_argument('-p', '--port', type=int, default=10006)
    parser.add_argument('-v', '--verbose', type=bool, default=False)
    parser.add_argument('-s', '--save_log', type=str, default=None)

    args = parser.parse_args()

    try:
        main(args)
    except KeyboardInterrupt:
        print('Process finished successfully by user, terminating now...')
    except Exception as exception:
        print('An unexpected error occurred:')
        print(exception)