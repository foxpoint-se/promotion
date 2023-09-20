import sharp from "sharp";
import fs from "fs";
import path from "path";

const validFileExtensions = <const>["jpeg", "jpg", "webp", "png"];
const validFileFormats = <const>["jpeg", "webp", "png"];
type ValidFormat = (typeof validFileFormats)[number];

const validFileExtensionStrings = validFileExtensions.map((e) => e as string);
const hasValidFileExtension = (fileName: string): boolean => {
  const extension = fileName.split(".").pop();
  return !!extension && validFileExtensionStrings.includes(extension);
};

const getFormatToUse = (
  format: keyof sharp.FormatEnum | undefined
): ValidFormat => {
  if (format === "png") {
    return "png";
  } else if (format === "webp") {
    return "webp";
  } else if (format === "jpeg") {
    return "jpeg";
  }
  throw new Error("Unsupported format: " + format);
};

const fileSuffix = "_COMPRESSED";

const notAlreadyCompressed = (pathToFile: string) =>
  !pathToFile.includes(fileSuffix);

type FormatConfig = {
  png: sharp.PngOptions;
  jpeg: sharp.JpegOptions;
  webp: sharp.WebpOptions;
};

const config: FormatConfig = {
  jpeg: { quality: 80 },
  webp: { quality: 80 },
  png: { quality: 80 },
};

const compressOneFile = async (path: string): Promise<string> => {
  const image = sharp(path);
  const meta = await image.metadata();
  const { format } = meta;
  const formatToUse = getFormatToUse(format);
  const compressedFile = path + fileSuffix + "." + formatToUse;
  await image[formatToUse](config[formatToUse])
    .resize(1200)
    .toFile(compressedFile);

  return compressedFile;
};

const getMegabytes = (pathToFile: string) => {
  const stats = fs.statSync(pathToFile);
  const fileSizeInBytes = stats.size;
  const fileSizeInMegabytes =
    Math.round((fileSizeInBytes / (1024 * 1024)) * 100) / 100;
  return fileSizeInMegabytes;
};

const compressAllImages = async () => {
  try {
    const imagesDirectory = path.resolve(process.cwd(), "./images");
    const files = await fs.promises.readdir(
      path.resolve(process.cwd(), imagesDirectory)
    );

    const imageFiles = files.filter(hasValidFileExtension);
    const filesToProcess = imageFiles.filter(notAlreadyCompressed);

    console.log("Looking in", imagesDirectory);
    console.log("File extensions:", validFileExtensions.join(", "));
    console.log("Found", filesToProcess.length, "files to process");

    const compressStats: {
      filename: string;
      MB_before: number;
      MB_after: number;
    }[] = [];

    for (const file of filesToProcess) {
      const fromPath = path.join(imagesDirectory, file);
      process.stdout.write(`Compressing ${file}... `);
      const MB_before = getMegabytes(fromPath);
      const compressedFile = await compressOneFile(fromPath);
      const MB_after = getMegabytes(compressedFile);
      compressStats.push({
        filename: file,
        MB_after,
        MB_before,
      });

      process.stdout.write(`Done!\n`);
    }

    console.table(compressStats);
  } catch (error) {
    console.error("Something went wrong", error);
  }
};

compressAllImages();
