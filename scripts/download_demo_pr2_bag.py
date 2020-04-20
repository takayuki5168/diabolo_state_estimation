#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'diabolo_state_estimation'

    download_data(
        pkg_name=PKG,
        path='log/demo_pr2.bag',
        url='https://drive.google.com/uc?id=1lwlJwwH_eXGjptXoNTCaQzpHoSjHlWbk',
        md5='69cad68c3ce39fdb1c694d1dc4c405e2',
        extract=False,
    )


if __name__ == '__main__':
    main()
