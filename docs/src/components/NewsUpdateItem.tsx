import { uglySimpleDateFormatting } from "./uglyFormatDate";

export type Update = {
  title: string;
  date: Date;
  description: string;
  linkUrl: string;
  image?: {
    url: string;
    description: string;
  };
};

const fallbackImage = {
  url: "/images/foxpoint_logo_no_image.png",
  description: "No image, Foxpoint logo",
};

export const NewsUpdateItem = ({ update }: { update: Update }) => {
  const image = update.image || fallbackImage;
  return (
    <div className="card card-compact bg-base-100 shadow-xl">
      {image && (
        <figure className="max-h-80">
          <img src={image.url} alt={image.description} />
        </figure>
      )}
      <div className="card-body">
        <h3 className="card-title">
          <a href={update.linkUrl} className="link link-primary">
            {update.title}
          </a>
        </h3>
        <div className="badge badge-ghost">
          {uglySimpleDateFormatting(update.date)}
        </div>
        <p>{update.description}</p>
      </div>
    </div>
  );
};
