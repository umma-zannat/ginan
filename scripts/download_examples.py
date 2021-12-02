'''Downloads single tarball which has products, data and solutions dirs
to disk and extracts the content into examples dir
how-to-update with aws cli: aws s3 cp examples_aux.tar.gz s3://peanpod/aux/ --acl public-read --metadata md5checksum=[checksum as computed by get_checksum]'''

import argparse
import os
import tarfile
import urllib.request
import shutil
import base64
import hashlib

def parse_arguments():
    parser = argparse.ArgumentParser(description='Downloads single tarball which has products, data and solutions dirs\
to disk and extracts the content into examples dir')
    parser.add_argument('-c', '--checksum', action='store_true',help='print checksum to terminal and return 0')
    return parser.parse_args()


def get_checksum(path2file):
    with open(path2file,'rb') as file:
        filehash = hashlib.md5()
        while True:
            data = file.read(8 * 1024 * 1024)
            if len(data) == 0:
                break
            filehash.update(data)
    return base64.b64encode(filehash.digest())

def untar(file):
    with tarfile.open(file,"r:gz") as tar:
        destpath = os.path.dirname(file)
        print('Extracting {} to {}'.format(file,destpath))
        tar.extractall(path=destpath)
        
def download_examples_tar(url,relpath = '../examples/',only_checksum=False):
    '''relpath configures output path relative to the sctipt location'''
    destfile = os.path.basename(url)
    script_path = os.path.dirname(os.path.realpath(__file__))
    destfile = os.path.abspath(os.path.join(script_path,relpath,destfile))
    if os.path.exists(destfile):
        print('examples tarball found on disk. Validating...')
        with urllib.request.urlopen(url) as response:
            if response.status == 200:
                print('server says OK -> computing MD5 of the file found---')
                md5_checksum = get_checksum(destfile).decode()
                if only_checksum:
                    print(f"MD5 is '{md5_checksum}'. Exiting as -c flag provided.")
                    return 0
                print('requesting MD5 from the server---')
                md5_checksum_response =  response.getheader('x-amz-meta-md5checksum')
                if md5_checksum_response is not None: #md5 checksum exists on server
                    
                    if md5_checksum == md5_checksum_response:
                        print("MD5 verified, skipping the download step")
                        untar(destfile)
                    else:
                        print(f"MD5 checksum failed. Expected MD5 is '{md5_checksum}'. Force redownloading the file")
                        if os.path.exists(destfile): os.remove(destfile)
                else:
                    print(f'no MD5 checksum header on the server. Expected MD5 is {md5_checksum}. Force redownloading the file')
                    print(destfile)
                    if os.path.exists(destfile): os.remove(destfile)

    if not os.path.exists(destfile):
        with urllib.request.urlopen(url) as response:
            if response.status == 200:
                print('downloading from {} to {}'.format(url, destfile))
                with open(destfile, 'wb') as out_file: shutil.copyfileobj(response, out_file)
        untar(destfile)



if __name__ == "__main__":
    parsed_args = parse_arguments()

    url='https://peanpod.s3.ap-southeast-2.amazonaws.com/aux/examples_aux.tar.gz'
    download_examples_tar(url=url,only_checksum=parsed_args.checksum)