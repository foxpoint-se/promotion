// 1. Import utilities from `astro:content`
import { z, defineCollection } from "astro:content";
// 2. Define your collection(s)
const docsCollection = defineCollection({
  type: "content", // v2.5.0 and later
  schema: z.object({
    title: z.string(),
    description: z.optional(z.string()),
    tags: z.optional(z.array(z.string())),
  }),
});

const updatesCollection = defineCollection({
  type: "content",
  schema: z.object({
    title: z.string(),
    description: z.string(),
    date: z.date(),
    image: z.optional(
      z.object({
        url: z.string(),
        description: z.string(),
      })
    ),
  }),
});

// 3. Export a single `collections` object to register your collection(s)
//    This key should match your collection directory name in "src/content"
export const collections = {
  docs: docsCollection,
  updates: updatesCollection,
};
