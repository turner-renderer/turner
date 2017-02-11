#!/usr/bin/env python
import argparse
import b2.api
import json
import os
import requests


def upload_images(bucket, image_files):
    """
    Uploads all images to B2 bucket.

    :param image_files paths to images
    :return: yields files infos.
    """
    for image_file in image_files:
        target_name = os.path.basename(image_file)
        file_info = bucket.upload_local_file(image_file, target_name)
        yield file_info.as_dict()


def comment(b2_api, file_ids, pr):
    """
    Posts a review comment on pull request.

    :param b2_api B2 api object
    :param file_ids Identifies image files on B2
    :param pr Identifies pull request
    """
    image_urls = (
        b2_api.get_download_url_for_fileid(file_id) for file_id in file_ids)
    body = ' '.join(
        ['<img src="{}" width="100">'.format(url) for url in image_urls])

    url = (
        "https://api.github.com/repos/turner-renderer/turner/pulls/{}/reviews"
        .format(pr))
    payload = {'body': body, 'event': 'COMMENT'}
    headers = {'Accept': 'application/vnd.github.black-cat-preview+json'}
    auth = (os.getenv('GITHUB_USER'), os.getenv('GITHUB_TOKEN'))
    requests.post(url, data=json.dumps(payload), headers=headers, auth=auth)


def upload_and_comment(commit, pr, folder, bucket_name="turner"):
    """
    Upload images from folder to B2 and post review comment on pull request.

    :param commit Identifies commit for image
    :param pr Identifies pull request
    :param folder Directory that is scanned for rendered images
    :param bucket_name Name of target bucket
    """

    # Authenticate and get access to bucket
    b2_api = b2.api.B2Api()
    b2_api.authorize_account(
        'production', os.getenv('B2_ACCOUNT_ID'), os.getenv('B2_APP_KEY'))

    bucket = b2_api.get_bucket_by_name(bucket_name)

    # Upload all files that start with commit
    image_files = (
        os.path.join(folder, f)
        for f in os.listdir(folder) if f.startswith(commit))
    file_infos = upload_images(bucket, image_files)

    # Comment on PR
    if pr is not None:
        file_ids = (info['fileId'] for info in file_infos)
        comment(b2_api, file_ids, pr)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Upload image to B2 and comment on pull request.')
    parser.add_argument(
        '-c', '--commit', required=True, help="Commit hash identifying image.")
    parser.add_argument(
        '-pr', '--pull-request', required=False,
        help="Pull request that is commented if provided.")
    parser.add_argument('folder', help="Folder which includes images")

    args = parser.parse_args()
    upload_and_comment(args.commit, args.pull_request, args.folder)
