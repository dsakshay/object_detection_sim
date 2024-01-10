#! /bin/bash

echo "What is your name?"

read name

if [ $name ]; then 
	echo "$name sounds good to me."
else
	echo "Doesn't sound like anything to me"
fi
