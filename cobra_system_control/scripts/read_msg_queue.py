import time
from cobra_system_control.remote import remote_lookup, COBRA_MESSENGER_ID
from Pyro5.errors import NamingError


if __name__ == '__main__':
    while True:
        try:
            with remote_lookup(COBRA_MESSENGER_ID) as msg:
                print(msg.get())
        except NamingError:
            time.sleep(2)
