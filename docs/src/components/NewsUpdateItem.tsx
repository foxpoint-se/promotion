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

export const NewsUpdateItem = ({ update }: { update: Update }) => {
  return (
    <div className="card card-compact bg-base-100 shadow-xl">
      {update.image && (
        <figure className="max-h-80">
          <img src={update.image.url} alt={update.image.description} />
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
