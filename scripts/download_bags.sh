#!/bin/bash

output_folder=$1

if [ -z "${output_folder}" ]
then
      echo "Please specify an output folder."
else

  if [ -d "${output_folder}" ]; then
    echo "Directory already exists" ;
  else
    mkdir -p "${output_folder}";
    echo "${output_folder} directory is created"
  fi

  # Single
  # https://drive.google.com/file/d/1sdYbYkH2Qrg8NaeKLY2JI062C7xEQ9d0/view?usp=sharing
  fileId=1sdYbYkH2Qrg8NaeKLY2JI062C7xEQ9d0
  fileName=single.bag
  curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=${fileId}" > /dev/null
  code="$(awk '/_warning_/ {print $NF}' /tmp/cookie)"
  curl -Lb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=${code}&id=${fileId}" -o "${output_folder}"/${fileName}

  # Bundle2
  # https://drive.google.com/file/d/1GCWRiNVnnqivSmZYaAifNv99dRFaVrwH/view?usp=sharing
  fileId=1GCWRiNVnnqivSmZYaAifNv99dRFaVrwH
  fileName=bundle_2.bag
  curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=${fileId}" > /dev/null
  code="$(awk '/_warning_/ {print $NF}' /tmp/cookie)"
  curl -Lb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=${code}&id=${fileId}" -o "${output_folder}"/${fileName}

  # Bundle4
  # https://drive.google.com/file/d/1BKhmcvU4TxO-gtDxTm4TF6qVHUZXeFzT/view?usp=sharing
  fileId=1BKhmcvU4TxO-gtDxTm4TF6qVHUZXeFzT
  fileName=bundle_4.bag
  curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=${fileId}" > /dev/null
  code="$(awk '/_warning_/ {print $NF}' /tmp/cookie)"
  curl -Lb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=${code}&id=${fileId}" -o "${output_folder}"/${fileName}
fi