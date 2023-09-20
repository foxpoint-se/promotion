# Promotion Site

Foxpoint site for promotion, documentation and logs.

https://www.foxpoint.se

## Develop

```
make setup
make dev
```

## Deploy

```
make deploy
```

## Compress images

Images are not compressed automatically when deploying. In this project there is a help utility for doing that, so you don't have to use a photo editing tool or whatever. Follow these steps:

1. Place your images in the `./compress/images` folder.
1. From the terminal, run `make compress`.
1. You will now have compressed versions of each image in the same folder. The new files have got funky file names.
1. Rename the files to whatever you want.
1. Place them wherever they should be, like `./docs/public/images` or something.
1. Profit! 🥳
