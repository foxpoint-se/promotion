export const uglySimpleDateFormatting = (d: Date) => {
  return d.toISOString().substring(0, 10);
};
