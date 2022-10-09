### Docker task 
Explore the nginx docker image.

## learning outcome
By exploring the nginx docker image we can learn how to create and use dockerfiles. Editing exsisting docker images will allow us to make new images that creates docker containers which executes the task we want acived. 
With this task we want you to learn how to:
- Manipulate exsisting docker images.
- Create basic dockerfiles.
- Apply a dockerfile in a docker compose.
- Open a port on the container for network traffic.

## task
Create a docker container using a dockerfile and docker compose which displayes a webpage saying "Vortex!". Index.html contain the necessary code for createing the webpage, feel free to run the html code to see the final result.
hint: When a docker container want to send network traffic out of the container (say to localhost) you need to open a port for the traffic to move out of the container.

Do not be afrad of google, some posible google queries to get you started:
- basic nginx dockerfile
- docker port binding
Remember! if you want to see a webpage hosted on you local machin write "localhost:<YOUR-PORT>", try find a way to create your container so you do not need to spesify a port when accessing you website (only "localhost").